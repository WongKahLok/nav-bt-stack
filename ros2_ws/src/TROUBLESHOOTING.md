# Troubleshooting log

Investigations that aren't captured elsewhere in the plan docs — symptom, root cause,
fix, and what's still open. Newest first.

---

## Follow-up: `<frame_id>` fix didn't stick — gz-sim's companion point-cloud topic can't honor it

**Symptom:** after the `<frame_id>` fix below and a rebuild, the TF tree itself checked out
clean (`view_frames`/`rqt_tf_tree` showed a live `odom → base_link` and a correct static
chain down through `livox_frame` and `chassis_imu_link`), but `slam_toolbox` was still
dropping every scan — now against a *different* bogus frame:

```text
[async_slam_toolbox_node] Message Filter dropping message: frame 'r1/livox_frame/livox_lidar'
at time <t> for reason 'discarding message because the queue is full'
```

**Diagnosis:** the scoped name changed from `r1/base_footprint/livox_lidar` (original bug,
below) to `r1/livox_frame/livox_lidar`. That change is actually confirmation that the
`disableFixedJointLumping` fix worked — `livox_frame` is now preserved as its own TF entity
instead of being lumped into `base_link`/`base_footprint`. But the `<frame_id>livox_frame</frame_id>`
override still isn't reflected on the message `slam_toolbox` receives.

**Root cause: a gz-sim limitation, not a config mistake.** `gpu_lidar` sensors publish two
separate outputs: the primary `LaserScan`/range data on `<topic>` (which *does* honor
`<frame_id>`), and an auto-generated companion `PointCloudPacked` on `<topic>/points` — and
`gazebo.launch.py` bridges the latter (see its bridge-config comment). gz-sensors'
`GpuLidarSensor` builds that companion topic's `header.frame_id` directly from the entity's
scoped name (`<model>/<link>/<sensor>`) and ignores the SDF `<frame_id>` override entirely —
a known asymmetry between the two output codepaths. No amount of further xacro tweaking to
`<frame_id>` can fix `/points`' frame; it's structurally incapable of respecting that field.

**Fix (identified, not yet applied/verified — needs editing in `robocup_nav`, outside this
workspace):** since the sensor has a fixed, zero-offset mount (`<pose>0 0 0 0 0 0</pose>`),
alias the bogus scoped frame to the real one with a static identity transform in
`gazebo.launch.py`:

```python
Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='livox_lidar_frame_alias',
    arguments=[
        '--x', '0', '--y', '0', '--z', '0',
        '--yaw', '0', '--pitch', '0', '--roll', '0',
        '--frame-id', 'livox_frame',
        '--child-frame-id', 'r1/livox_frame/livox_lidar',
    ],
),
```

This doesn't make `slam_toolbox` "look at the parent" as a special case — a TF lookup
(`lookupTransform(target, source)`) always walks the whole tree from `source` up through
parent edges until it finds a path to `target`. Right now `r1/livox_frame/livox_lidar` has
*no* edge at all, so the walk dead-ends immediately and the message filter waits forever.
Adding this static publisher inserts exactly the one missing edge:

```text
r1/livox_frame/livox_lidar --(static, identity)--> livox_frame --(existing)--> base_link --(existing)--> ... --> odom / map
```

— after which the lookup succeeds via a normal walk over edges that were already working
(per the TF tree capture that motivated this entry). Same aliasing trick would apply to
`chassis_imu`'s companion topic if it turns out to have the same asymmetry — not yet
checked, since `ekf_node` consuming `/imu` hasn't shown the equivalent symptom.

**Status: fix identified, not yet applied.** `gazebo.launch.py` lives in the separate
`robocup_nav` workspace, not this repo, so it couldn't be edited/verified from here. Next:
add the static transform publisher above to `robocup_nav`'s `gazebo.launch.py`, relaunch,
and re-check `ros2 topic echo /livox/lidar --field header.frame_id --once` — expect it to
still read `r1/livox_frame/livox_lidar` (that's fine, the alias doesn't change the topic,
just gives TF an edge for it) and confirm `slam_toolbox` stops dropping messages / `map`
frame appears.

---

## Livox point cloud / chassis IMU `frame_id` not resolving in TF — empty rviz, scans never fused

**Symptom:** ran `ros2 launch nav_bt_stack slam_launch.py` against a recorded bag; rviz2
showed nothing after adding the map/LaserScan displays. Log was full of repeating:

```text
[async_slam_toolbox_node] Message Filter dropping message: frame 'r1/base_footprint/livox_lidar'
at time <t> for reason 'discarding message because the queue is full'
```

**Diagnosis commands:**

```bash
# Dump the live TF tree while the bag/sim is running
ros2 run tf2_tools view_frames
# or, just the static ones
ros2 topic echo /tf_static --once

# Check what frame_id is actually stamped on the raw sensor message
ros2 topic echo /livox/lidar --field header.frame_id --once
```

`view_frames` showed a normal tree — `livox_frame` present, parented to `base_link`,
`base_footprint` as root — with **no frame named `r1/base_footprint/livox_lidar`
anywhere**. That string wasn't "not yet available," it was never going to resolve: it
doesn't correspond to any link ever broadcast to TF.

**Root cause:** in `robocup_nav/ros2_ws/src/galaxea_simulation/urdf/r1_sensors.urdf.xacro`,
the `livox_lidar` (`gpu_lidar`) and `chassis_imu` (`imu`) Gazebo sensor blocks had no
explicit `<frame_id>` tag. Without one, gz-sim's sensor plugins auto-generate the
published message's `header.frame_id` from the sensor's *scoped entity path*
(`<model>/<canonical_link>/<sensor_name>`, e.g. `r1/base_footprint/livox_lidar`) instead
of using the URDF link name the `<gazebo reference="...">` block is actually attached to
(`livox_frame`). `pointcloud_to_laserscan` copies that bogus frame straight onto `/scan`,
so slam_toolbox's `tf2_ros::MessageFilter` waits forever for a transform that will never
exist, the queue fills, and every scan gets silently dropped — nothing ever reaches the
map. The chassis IMU sensor had the identical gap (no `<frame_id>chassis_imu_link</frame_id>`),
which would cause `ekf_node` to silently drop IMU fusion the same way (TF lookup failure,
not a crash, so it's easy to miss).

**Fix applied** (`robocup_nav/ros2_ws/src/galaxea_simulation/urdf/r1_sensors.urdf.xacro`):
added the missing tag to both sensor blocks so published frame_ids match what
`robot_state_publisher` actually broadcasts:

```xml
<sensor name="livox_lidar" type="gpu_lidar">
  <frame_id>livox_frame</frame_id>
  ...
</sensor>
...
<sensor name="chassis_imu" type="imu">
  <frame_id>chassis_imu_link</frame_id>
  ...
</sensor>
```

**Status: fix applied, not yet verified.** Next: rebuild `galaxea_simulation`, relaunch,
re-check `ros2 topic echo /livox/lidar --field header.frame_id --once` reads
`livox_frame`, and confirm scans appear in rviz with no more queue-full drops.

---

## EKF NaN output (`TF_NAN_INPUT` / `TF_DENORMALIZED_QUATERNION`) right after launch

**Symptom:** immediately after `ros2 launch nav_bt_stack slam_launch.py` starts, before
any real motion, `ekf_node` logs repeat:

```text
[ekf_node] Critical Error, NaNs were detected in the output state of the filter. This was
likely due to poorly coniditioned process, noise, or sensor covariances.
[ekf_node] Error: TF_NAN_INPUT: Ignoring transform for child_frame_id "base_link" ...
[ekf_node] Error: TF_DENORMALIZED_QUATERNION: Ignoring transform for child_frame_id "base_link" ...
```

**Root cause:** `nav_bt_stack/scripts/imu_hdas_translator.py` built the outgoing
`sensor_msgs/Imu` but never set `orientation_covariance`, `angular_velocity_covariance`,
or `linear_acceleration_covariance` — they default to all-zero. Per ROS/REP-103
convention, all-zero covariance means "measured with *zero* uncertainty" (only a leading
`-1` means "no estimate available"). `nav_bt_stack/config/ekf.yaml`'s `imu0_config` fuses
yaw, vyaw, ax, ay from `/imu`, so `ekf_node` was treating those as perfectly certain —
combined with `ekf.yaml`'s own near-zero `initial_estimate_covariance` (`1e-9`), the
filter's info matrix became ill-conditioned and blew up to NaN almost immediately. The
error message's own hint ("sensor covariances") pointed straight at it.

**Fix applied** (`nav_bt_stack/scripts/imu_hdas_translator.py`): set placeholder diagonal
covariances on the published `Imu` message (`hdas_msg/Imu` carries no noise/covariance
fields at all, so there's nothing to propagate from upstream — same "placeholder, tune
once real sensor noise is characterized" status as `swerve_odometry.py`'s odom
covariance):

```python
out.orientation_covariance = _ORIENTATION_COVARIANCE
out.angular_velocity_covariance = _ANGULAR_VELOCITY_COVARIANCE
out.linear_acceleration_covariance = _LINEAR_ACCELERATION_COVARIANCE
```

**Verified fixed:** re-ran `ros2 launch nav_bt_stack slam_launch.py` — no more
`Critical Error`/`TF_NAN_INPUT`/`TF_DENORMALIZED_QUATERNION` lines in the log.

---

## Swerve odometry heading error during strafe (`vy`) commands

**Symptom:** driving `/model/r1/pose` (Gazebo ground truth) against `/odom`
(`swerve_odometry.py`, in `robocup_nav`) in Foxglove during the `pure vy` test from
`commands.md`, `/odom.pose.pose.position.x` drifted away from ground truth even though
the commanded motion was pure lateral (`vy` only, no `vx`/`wz`) — ground truth `x` stayed
flat, `/odom`'s `x` ramped linearly. Position `y` looked fine; only `x` (and, once checked
directly, heading) showed the problem.

**Root cause:** `swerve_controller.py`'s `_control_loop` (in `robocup_nav`'s
`galaxea_simulation`) computes each module's target steer angle and immediately commands
the wheel at full target speed, without waiting for the physical steering motor to reach
that angle. Whenever the commanded motion's *shape* changes (e.g. going from stationary to
strafing forces all three modules to re-steer ~90°), there's a transient window where the
wheels are spinning at full speed while still pointed in roughly their old direction. During
that window, `swerve_odometry.py`'s least-squares FK (`scripts/swerve_odometry.py`, reading
back actual `/joint_states` angle + velocity) sees three module ground-velocity vectors that
don't correspond to any single consistent rigid-body twist, and can hand back a `wz` with
the wrong magnitude — and in one confirmed case, the wrong *sign*.

Because `self.theta` in `swerve_odometry.py` is integrated open-loop
(`self.theta += wz * dt`, never corrected), a bad `wz` during one transient becomes a
**permanent constant heading bias** afterward. From then on, the position integration
step (`self.x += (vx*cos(theta) - vy*sin(theta)) * dt`) leaks a constant fraction of `vy`
into `x` for as long as `vy` stays nonzero — which is exactly the linear-looking `x` drift
that was originally observed. (`y` didn't show an equivalent obvious artifact because
`cos(small theta) ≈ 1`, so the leak into `y` is second-order and much smaller.)

**Confirmed via:** pulled `/model/r1/pose` and `/odom` directly out of the recorded
`.mcap` bag (`ros2_ws/src/rosbags/ros2_output/ros2_output_0.mcap`) with the `mcap`/
`mcap-ros2-support` Python libs. Two checks:
- `/odom.pose.pose.orientation.z` held a constant ~0.081 (yaw ≈ 9.3°) through a window where
  ground truth held flat at ~0.011 (yaw ≈ 1.3°) — the "stuck bias" signature.
- Directly compared `/odom.twist.twist.angular.z` (the FK's instantaneous, non-integrated
  `wz`) against a finite-differenced ground-truth yaw rate
  (`(yaw[i]-yaw[i-1])/(t[i]-t[i-1])`) in the transition window: ground truth was clearly
  positive (chassis actually rotating the commanded direction) while `/odom`'s `wz` was
  clearly negative for the same interval — confirms the FK sign flip directly, not just an
  inference from accumulated drift.

**Fix applied** (`robocup_nav/ros2_ws/src/galaxea_simulation/scripts/swerve_controller.py`,
`_control_loop`): cosine-compensate wheel speed by the module's remaining steering error
after the 180°-flip decision:

```python
delta = self._normalize(angle - self.current_steer[i])
if abs(delta) > math.pi / 2.0:
    angle = self._normalize(angle + math.pi)
    wheel_vel = -wheel_vel
    delta = self._normalize(angle - self.current_steer[i])

wheel_vel *= max(0.0, math.cos(delta))
```

A module pointed exactly at its target drives at full speed; one still mid-turn drives
proportionally slower; one 90° off drives at zero. This is the standard "cosine
compensation" technique for swerve drives.

**Verified fixed:** re-recorded the same `pure vy` test. At the re-steer transient, ground
truth and `/odom`'s `wz` are both ~0 the whole window (no more sign flip). Heading error at
steady state dropped from a locked ~8-9° to ~2°.

---

## Residual heading drift during sustained high-speed strafe — real wheel slip, not a bug

**Symptom:** even after the fix above, a `vy=0.6` strafe held for ~10+ continuous seconds
(well past any re-steer transient, modules already settled at their target angle) showed
ground truth (`/model/r1/pose`) itself drifting off 0° yaw — up to a dramatic spin-out
(+9° to -38° in ~2s) in one run, a gentler steady creep (down to -6.5° over ~10s) in
another.

**Root cause: not a software bug.** The *ground truth* pose is the one drifting, not just
`/odom` failing to track a stationary robot — the physical chassis is genuinely rotating.
Likely wheel slip/traction behavior in the sim at `vy=0.6` (3x the `0.2` test that tracked
cleanly). Wheel-encoder-based odometry is structurally blind to this: encoders measure how
fast the wheels spin, not whether the ground let them grip, so no amount of fixing the FK
math in `swerve_odometry.py` can close this gap.

**Plan:** this is exactly what the `/imu`-fused EKF (`ekf_node`, wired into
`nav_bt_stack/launch/bringup_launch.py`, configured by `nav_bt_stack/config/ekf.yaml`) is
for — the chassis IMU's gyro measures actual chassis rotation directly, independent of
wheel slip, so fusing it should pull the heading estimate back toward truth even when the
wheels are lying about it via slip. Confirmed as the right next step, not a workaround.

**Watch for when tuning:** `config/ekf.yaml`'s `process_noise_covariance` and `/odom`'s
pose/twist covariance (set in `swerve_odometry.py`) are still stock/placeholder values —
both have comments noting they're waiting on real slip data to tune. If the EKF still
leans too hard on wheel odometry's yaw during high-slip strafes like this, that's the
parameter to revisit (see `localization_slam_tuning_plan.md` Step 1).

---

## Reference: orientation vs. angular velocity, for future debugging

Useful distinction when diffing `/odom` against ground truth:

- **Orientation** (`pose.orientation`, quaternion) is a *position*-like quantity — "which
  way is it facing right now." For a robot that only yaws (true here, `two_d_mode: true`
  in the EKF), `orientation.z ≈ sin(yaw/2)` and `orientation.w ≈ cos(yaw/2)`, so small `z`
  is a usable proxy for small yaw.
- **Angular velocity** (`twist.angular.z`, `wz`) is a *rate* — how fast that heading is
  changing right now. It's the derivative of yaw, same relationship as `vx` to `x`.
- `swerve_odometry.py` integrates `wz` into `theta` every cycle with no correction, so
  **orientation shows accumulated damage** (a bias that appeared once looks the same as
  one that's been growing the whole time), while **angular velocity shows the instant it
  happened** (each sample is independent, no memory of previous cycles). When chasing a
  heading bug, check the rate first to localize *when* it went wrong, then check the
  integrated orientation to see the resulting damage.
- Ground truth (`/model/r1/pose`) only publishes orientation, not a rate — to compare
  rates apples-to-apples against `/odom.twist.twist.angular.z`, finite-difference
  consecutive ground-truth yaw samples: `(yaw[i]-yaw[i-1]) / (t[i]-t[i-1])`.
