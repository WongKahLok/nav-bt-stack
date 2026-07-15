# Two-repo Nav2 bring-up: lidar + odometry + IMU in the sim, params + launch + fusion in nav_bt_stack

## Context

`nav_bt_stack` had a tuned `nav2_params.yaml` but no launch file and no robot model, so
nothing published the TF/topics its BT leaves already assume. Investigating turned up a
second, existing repo — `/home/bremen/robocup_nav` (package `galaxea_simulation`, a Gazebo
Fortress sim of the real robot: Galaxea R1, swerve-drive base + dual arm) — with a real,
CAD-exported URDF (`urdf/r1_v2_1_0.urdf`) and a working `swerve_controller.py` that turns
`/cmd_vel` into per-module steer+wheel commands. So the robot model already exists; it
should not be re-authored inside `nav_bt_stack`.

Verified directly against the source files (not just summarized): the sim's real frame
names (`base_footprint`/`base_link`, `zed_link`, `left_arm_base_link`/`right_arm_base_link`,
`left_realsense_link`/`right_realsense_link`) don't match what `nav_bt_stack`'s own docs
assumed (`base_scan`, `camera_link`, singular `arm_base_link`, `tool0`) — those docs were
written aspirationally before this sim existed. More importantly, two hard blockers exist
before Nav2 can run against this sim at all: **no lidar** anywhere in the URDF/Gazebo
config, and **no odometry** (`r1_gazebo_sim_plan.md` explicitly flags swerve
forward-kinematics → `/odom` as planned-but-not-built; only `/joint_states` currently
publishes).

**Real-hardware sensor facts (grounded against the actual firmware install tree,
`atc_standard-V2.0.4-20250516_21_53_33_aarch64_patch1/install/`, not guessed):**

- Lidar is a **Livox Mid-360** (`livox_ros_driver2` is installed, with a
  `MID360_config.json` and a `msg_MID360_launch.py`). The real driver publishes
  `sensor_msgs/msg/PointCloud2` (`xfer_format: 0` = `PointXYZRTL`, not a custom format),
  `frame_id: livox_frame`, on the driver's standard topic (commonly `/livox/lidar` for this
  driver — confirm empirically via `ros2 topic list` against real hardware before treating
  as final, same rigor the file already applies to Ignition topic names below). It is
  **not** a native 2D scanning lidar — there is no `sensor_msgs/LaserScan` anywhere in this
  driver's output. The originally-assumed "RPLIDAR S3, native `/scan`" in Part 1 below was
  wrong and is corrected.
- IMU: HDAS exposes **two separate, physically distinct** IMUs, not one signal under two
  names — confirmed via `HDAS/share/HDAS/launch/r1.py`'s two independent topic args:
  `chassis_imu_feedback_topic_name` → `/hdas/imu_chassis`, and
  `torso_imu_feedback_topic_name` → `/hdas/imu_torso`. `/hdas/imu_chassis` is rigidly
  mounted on the base; `/hdas/imu_torso` sits downstream of the articulated `torso_joint2-4`,
  so its orientation relative to `base_link` changes whenever the torso moves. Since this
  project drives the base only (no torso/arm motion during nav), **`/hdas/imu_chassis` is
  the correct source**, independent of the torso-motion caveat — it's also just the
  base-fixed one.
- The real `/hdas/imu_chassis` message type is **`hdas_msg/msg/Imu`** (`roll, pitch, yaw,
  groy_x/y/z, acc_x/y/z`) — a custom HDAS type, **not** `sensor_msgs/msg/Imu`. Gazebo's IMU
  plugin only ever emits standard `sensor_msgs/msg/Imu` (that's all `ros_gz_bridge`'s
  built-in mapping table supports), so matching the real type exactly requires a small
  translator inside `robocup_nav` (Part 2) that converts the bridged `sensor_msgs/Imu` into
  `hdas_msg/Imu` and republishes it on `/hdas/imu_chassis` — **decision: do this**, so that
  `nav_bt_stack`'s own `hdas_msg/Imu → sensor_msgs/Imu` translator (Part 7) is a single,
  always-on node that runs identically against sim or real hardware, rather than a
  real-hardware-only special case.
- No AMCL, `robot_localization`, Cartographer, or NDT reference exists anywhere in the
  firmware tree — the vendor stack does not ship a localization solution. That confirms
  AMCL + `robot_localization` living entirely in `nav_bt_stack` is correct, not a gap to
  chase down elsewhere.

**Architecture decision:** keep the two repos as two separate containers/stacks, integrated
over the ROS graph rather than merged into one workspace.

- `robocup_nav` has two git remotes — `origin` → `Team-Robo/galaxear1-sim` (shared team
  repo) and `personal` → a fork. `nav_bt_stack` is solely personal. Merging would mean
  vendoring one into the other and manually tracking drift.
- The real robot deployment runs `nav_bt_stack` with no Gazebo at all; `robocup_nav`'s
  container carries the full Gazebo Fortress + GPU/X11 stack. Keeping them separate means
  the real-robot path never has to drag the sim container along.
- No code-level coupling exists today — only a runtime ROS topic/TF contract, which is
  exactly what separate containers on one ROS graph are for. This matches the general
  pattern across the ROS ecosystem (description/sim packages stay separate from the
  nav-stack packages that consume them) so other future consumers (manipulation,
  perception) aren't forced to depend on the nav repo just to get the URDF.
- Confirmed compatible with zero networking changes: both `docker-compose.yml` files use
  `network_mode: "host"`, and both default to `ROS_DOMAIN_ID=0` (`nav_bt_stack` sets it
  explicitly; `robocup_nav` doesn't set it, so it also defaults to 0) — DDS discovery
  already works across the two containers on the same Docker host.
- **Ownership split, restated precisely now that IMU is in scope:** `robocup_nav` owns the
  robot description/sim and publishes exactly what a real robot's raw drivers would publish
  — `/joint_states`, `/livox/lidar` (PointCloud2), `/hdas/imu_chassis` (`hdas_msg/Imu`,
  matching the real type exactly — see the IMU bullet above), `/odom` (raw wheel
  forward-kinematics, no fusion). `nav_bt_stack` owns everything that *estimates or plans*
  from those raw topics — `pointcloud_to_laserscan`, the `hdas_msg/Imu → sensor_msgs/Imu`
  translator, the `robot_localization` EKF, AMCL, costmaps, planner/controller/behavior
  servers, `bt_navigator`, the BT stack. This corrects the earlier version of this doc,
  which said the EKF would live on the `robocup_nav` side "since the sim side now owns all
  of that" — an EKF is an estimation component, not a sensor publisher, so it belongs with
  the rest of `nav_bt_stack`'s consumer-side logic, matching the pattern already
  established for AMCL/SLAM/nav2_params.

**Status:** Parts 3-5 (the `nav_bt_stack` side, original scope) are implemented. Parts 1-2
(`robocup_nav`) and Parts 6-7 (`nav_bt_stack`, newly added now that IMU is in scope) are
documented below but **not yet implemented** — `robocup_nav` is a shared team repo
(`Team-Robo/galaxear1-sim`), so that work is deliberately left for a separate
pass/coordination rather than done inline here.

**Scope guardrail:** depth-camera sensor plugins, arm/manipulation modeling, and a real
arena-like world (vs. the current empty world) are explicitly deferred (see bottom) — not
needed to unblock Nav2. IMU + EKF fusion is **no longer deferred** (Parts 2, 7 below) now
that a concrete real-hardware IMU source has been identified and there's a structural
reason (torso articulation) to want it regardless of hardware precedent.

---

## Part 1 (not yet implemented) — `robocup_nav`: add a lidar sensor

The exported URDF (`urdf/r1_v2_1_0.urdf`) is a CAD tool export (SolidWorks-style sidecar
CSV, STEP-file component name breadcrumbs) that will likely be regenerated wholesale on
the next re-export — don't hand-edit it. Instead:

- **New file** `urdf/r1_sensors.urdf.xacro`: `<xacro:include filename="r1_v2_1_0.urdf"/>`,
  then add a `livox_frame` link with a fixed joint off `base_link`, mounted on the **left
  side** (per hardware, single lidar — the spec sheet's dual front-corner mount is optional
  and not what's being modeled). Placeholder mount xyz/rpy, commented for tuning against the
  real mount point, matching the existing "tune to real chassis" comment style already used
  in `nav_bt_stack/config/nav2_params.yaml`.
- Sensor: `<gazebo reference="livox_frame"><sensor type="gpu_lidar">`, configured to emulate
  a **Livox Mid-360** (not the previously-assumed RPLIDAR S3): 360° horizontal, ~59°
  vertical FOV, publishing `sensor_msgs/msg/PointCloud2` — matching the real driver's
  contract (`frame_id: livox_frame`, PointCloud2, confirmed above) rather than a native
  `sensor_msgs/LaserScan`. A modest vertical sample count (a handful of rings) is enough;
  exactly replicating the Mid-360's non-repetitive scan pattern isn't achievable in Gazebo
  and isn't needed for 2D nav.
- **2D reduction happens downstream, not here**: don't fake a native 2D LaserScan sensor in
  Gazebo. Publishing the real PointCloud2 contract and doing the 3D→2D reduction via
  `pointcloud_to_laserscan` (Part 6, `nav_bt_stack`) means the exact same conversion node
  and config work unmodified against real hardware later — the real driver also only emits
  PointCloud2, so a sim that skips straight to a fake `/scan` would diverge from what
  `nav_bt_stack` will actually have to handle on the physical robot.
- **Update** `launch/gazebo.launch.py`: currently `robot_description` is a plain file
  read of the raw `.urdf` (lines 12-16). Change it to run the new `r1_sensors.urdf.xacro`
  through `xacro` (`Command(['xacro ', path])`) instead. Add a `ros_gz_bridge` argument +
  remapping for the lidar's Ignition topic → ROS `/livox/lidar`
  (`sensor_msgs/msg/PointCloud2`), matching the real driver's topic name so `nav_bt_stack`
  never needs to know whether it's talking to sim or real hardware, mirroring the existing
  joint_state bridge/remap pattern (`gazebo.launch.py:54-67`).
  - Flag: confirm the exact auto-generated Ignition topic name empirically at runtime via
    `ign topic -l` before finalizing the bridge argument — the file's own existing comment
    (`gazebo.launch.py:49`) already calls out this exact check for the same reason.
- `package.xml` / `Dockerfile`: add `xacro` as a dependency (not currently listed).

## Part 2 (not yet implemented) — `robocup_nav`: swerve forward-kinematics odometry + chassis IMU

**New file** `scripts/swerve_odometry.py`, installed the same way as the existing
`swerve_controller.py` (mirror its `MODULES` list and `wheel_radius` default — 0.07 m —
exactly, since there's no shared constants module in this package and it's not worth
introducing one for three tuples of four numbers).

- Subscribe to `/joint_states`. For each module, wheel velocity vector in body frame is
  `wheel_radius * wheel_velocity * (cos(steer_angle), sin(steer_angle))` — the inverse of
  the IK `swerve_controller.py` already does (`swerve_controller.py:108-123`).
- Stack the 3 modules' velocity vectors into a small least-squares solve
  (`numpy.linalg.lstsq`) for body-frame `(vx, vy, wz)`.
- Euler-integrate over `dt` to track `(x, y, theta)`; publish `nav_msgs/msg/Odometry` on
  `/odom` with non-zero covariance (so the downstream EKF can weight it). `child_frame_id:
  base_link` — matching `nav2_params.yaml`'s AMCL `base_frame_id: base_link` (Part 3,
  already implemented — that's the settled choice, not `base_footprint`; see the frame-name
  note at the end of this part).
- **Do not broadcast `odom → base_link` TF from this node.** Once Part 7's EKF exists
  in `nav_bt_stack`, the EKF is the single owner of that transform — two nodes publishing
  the same TF is a real bug, not a redundancy. This node publishes the `/odom` topic only.

**Chassis IMU** — add a `chassis_imu_link` to `r1_sensors.urdf.xacro` (Part 1), fixed to
`base_link` (not `torso_imu`-equivalent — see the real-hardware IMU findings above: the
torso-mounted one is wrong for base localization since torso joints articulate).
`<gazebo reference="chassis_imu_link"><sensor type="imu">`, bridged via `ros_gz_bridge` to
an internal-only ROS topic (e.g. `/_gz/imu_chassis_raw`) as `sensor_msgs/msg/Imu` — that's
the only type `ros_gz_bridge` can produce from Gazebo's IMU plugin.

**IMU type translator**, new small node (`scripts/imu_hdas_translator.py` or folded into
`swerve_odometry.py`'s process): subscribes the bridge's `sensor_msgs/Imu`, republishes as
`hdas_msg/msg/Imu` on `/hdas/imu_chassis` — direct copies for `angular_velocity` →
`groy_x/y/z` and `linear_acceleration` → `acc_x/y/z`, quaternion → Euler for
`roll/pitch/yaw`. This makes `/hdas/imu_chassis` match the real robot's exact topic name
and message type, so `nav_bt_stack`'s reverse translator (Part 7) needs no sim/real
branching.

- New build dependency: `hdas_msg` (from the `atc_standard` install tree, not a standard
  ROS distro package — confirm how it's sourced/overlaid before adding to `package.xml`,
  since it isn't vendored into this repo today).

- Add as new `Node(...)` entries in `gazebo.launch.py` (same pattern as
  `swerve_controller` at `gazebo.launch.py:71-77`), and add
  `scripts/swerve_odometry.py` to `CMakeLists.txt`'s `install(PROGRAMS ...)` block
  alongside the existing entry.

## Part 3 (implemented) — `nav_bt_stack`: merge `config/nav2_params.yaml` against the stock reference

`config/copy-nav2-params.yaml` (the stock TB3 reference) and `config/nav2_params.yaml`
already share the same section order for every section they have in common — this is an
insertion job, not a reshuffle.

- After `bt_navigator`: add `bt_navigator_navigate_through_poses_rclcpp_node` and
  `bt_navigator_navigate_to_pose_rclcpp_node` (`use_sim_time` only each), and add
  `wait_for_service_timeout: 1000` to `bt_navigator` itself.
- `amcl`: add the likelihood-field/beam-model + KLD-sampling keys that are missing —
  `beam_skip_distance`, `beam_skip_error_threshold`, `beam_skip_threshold`, `do_beamskip`,
  `lambda_short`, `laser_likelihood_max_dist`, `laser_model_type`, `pf_err`, `pf_z`,
  `recovery_alpha_fast`, `recovery_alpha_slow`, `save_pose_rate`, `sigma_hit`,
  `tf_broadcast`, `z_hit`, `z_max`, `z_rand`, `z_short`. Keep `base_frame_id: base_link`,
  `robot_model_type: OmniMotionModel`, `laser_max_range: 12.0` as-is — deliberate. No
  changes needed here for the Part 6 `pointcloud_to_laserscan` addition: AMCL still just
  consumes a `LaserScan` on its configured `scan_topic`, it doesn't care that the scan is
  now synthesized from a point cloud upstream instead of coming from a native 2D lidar.
- `controller_server.FollowPath` (the DWB block under the RotationShim): add
  `linear_granularity`, `angular_granularity`, `xy_goal_tolerance`,
  `trans_stopped_velocity`, `short_circuit_trajectory_evaluation`, `stateful`. Keep the
  `RotationShimController` wrapper and all vel/accel/critic values as-is.
- After `map_server`: add `map_saver`. After `behavior_server`: add
  `robot_state_publisher` (`use_sim_time` only — the node itself now runs in
  `robocup_nav`, this is just the parameter block Nav2's own launch expects to exist) and
  `waypoint_follower` (copy stock block as-is).
- `velocity_smoother`: add `deadband_velocity` and `velocity_timeout`.
- `bt_navigator.plugin_lib_names`: leave as-is. Stock lists 47 vs the current 14; the
  other 33 include BT-node plugins newer than a base Humble install and may not exist as
  `.so` files in this container — don't bulk-copy, add specific entries later only if a
  tree XML references one and fails to load.
- Leave `planner_server` (`SmacPlannerHybrid`), the `RotationShimController` wrapper,
  `local_costmap`'s split `obstacle_layer`+`voxel_layer`, and all radius/inflation tuning
  untouched — deliberate departures from the TB3 stock file, not omissions.
- Delete `config/copy-nav2-params.yaml` once merged (scratch reference, nothing loads it).

## Part 4 (implemented) — `nav_bt_stack`: `launch/bringup_launch.py` (new)

A pure consumer launch file — no robot_state_publisher, no xacro (that's `robocup_nav`'s
job, Parts 1-2 above). **Now also needs Parts 6-7 wired in** (see those parts) — flagging
here since this file will need a follow-up edit once Parts 6-7 land, not a full rewrite:

1. Declare args: `use_sim_time` (default `true`, since this is exercised against the sim
   first), `map` (yaml path), `params_file` (default: the installed `nav2_params.yaml` via
   `PathJoinSubstitution([FindPackageShare("nav_bt_stack"), "config", "nav2_params.yaml"])`).
2. `IncludeLaunchDescription` of `nav2_bringup`'s `bringup_launch.py` (stays in the
   `nav2_bringup` package, not vendored), passing through `params_file`, `map`,
   `use_sim_time`. This brings up AMCL, controller/planner/behavior servers, `bt_navigator`,
   `waypoint_follower`, `velocity_smoother`, lifecycle manager — all already configured via
   Part 3's `nav2_params.yaml`.
3. The `bt_main` node (this package's own executable).
4. **(new, Part 6)** `pointcloud_to_laserscan` node, consuming `/livox/lidar`.
5. **(new, Part 7)** `robot_localization` `ekf_node`, consuming `/odom` + `/imu`.

- `CMakeLists.txt`: add `install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})`,
  following the exact pattern already used for `bt_xml`/`config`
  (`CMakeLists.txt:47-51`).
- `package.xml`: add `<exec_depend>` for `nav2_bringup`, `launch`, `launch_ros`, and (Parts
  6-7) `pointcloud_to_laserscan`, `robot_localization`.
- Known limitation to flag, not fix here: `robocup_nav`'s current world
  (`worlds/empty.sdf`) has no obstacles, so AMCL/SLAM will have very little to actually
  localize against — a real arena-like world (with the 915 mm door) is a separate
  follow-up.

## Part 5 (implemented) — `nav_bt_stack`: Dockerfile / docker-compose cleanup

- `Dockerfile`: remove `ENV TURTLEBOT3_MODEL=burger` and
  `ros-humble-turtlebot3-navigation2` — this robot isn't a TurtleBot3, it's leftover from
  bootstrapping.
- `docker-compose.yml`: remove the now-unused `TURTLEBOT3_MODEL: burger` environment entry.

## Part 6 (not yet implemented) — `nav_bt_stack`: `pointcloud_to_laserscan` bridge

Needed because neither the sim (Part 1) nor the real Livox Mid-360 driver emit a native
`LaserScan` — both only emit `PointCloud2` on `/livox/lidar`. AMCL and the costmaps in
`nav2_params.yaml` (Part 3) are already configured to consume a `LaserScan`, unchanged.

- Add a `pointcloud_to_laserscan` node (`pointcloud_to_laserscan_node` from the
  `pointcloud_to_laserscan` package) subscribing `/livox/lidar`, publishing `/scan`
  (`sensor_msgs/msg/LaserScan`). Leave `target_frame` unset (defaults to the input cloud's
  own frame, `livox_frame`) rather than forcing it to `base_link` — AMCL and the costmap
  layers already TF-transform each `LaserScan` reading from its sensor frame into
  `base_link`/`map` at consume time via the static `base_link → livox_frame` transform
  (published by `robot_state_publisher` from the URDF), so there's no need for
  `pointcloud_to_laserscan` to do that transform itself.
  - Range/angle limits (`min_height`/`max_height` for the vertical slice, `range_min`,
    `range_max`) should match `nav2_params.yaml`'s existing `laser_max_range: 12.0` and the
    Mid-360's real max range — don't leave these at the package's stock defaults
    unreviewed.
- Add to `launch/bringup_launch.py` (Part 4 item 4) and `package.xml`.
- This is identical whether consuming sim or real hardware — no sim-specific branching
  needed, which is the whole point of matching the real topic/type contract in Part 1.

## Part 7 (not yet implemented) — `nav_bt_stack`: IMU translator + `robot_localization` EKF (odom + IMU fusion)

- **IMU translator** (new small node, e.g. `imu_hdas_translator.py`): subscribes
  `/hdas/imu_chassis` (`hdas_msg/msg/Imu` — `roll, pitch, yaw, groy_x/y/z, acc_x/y/z`,
  identical whether it's `robocup_nav`'s sim translator (Part 2) or the real HDAS driver
  publishing it), converts to `sensor_msgs/msg/Imu` (quaternion from roll/pitch/yaw,
  `angular_velocity` from `groy_x/y/z`, `linear_acceleration` from `acc_x/y/z`), republishes
  on `/imu`. Because `robocup_nav` now emits the real `hdas_msg/Imu` type on the real topic
  name (Part 2's decision), **this translator is a single always-on node, not a
  real-hardware-only special case** — it runs unmodified against sim and real. This is a
  genuine translator, not a pure remap (unlike the chassis-command case in
  `r1_sim_plan_addendum.md`, which turned out to be a remap because the types already
  matched) — the types differ here, so an actual conversion node is required.
- New `config/ekf.yaml`: fuse `/odom` (`nav_msgs/Odometry`, wheel FK from Part 2) and `/imu`
  (`sensor_msgs/Imu`, from the translator above) into `/odometry/filtered`.
  `odom_frame: odom`, `base_link_frame: base_link`, `world_frame: odom` (this EKF only
  estimates `odom → base_link`; AMCL owns `map → odom` separately, standard Nav2
  two-EKF-less layering — no second EKF needed here). `base_link`, not `base_footprint` —
  matches `nav2_params.yaml`'s AMCL `base_frame_id: base_link` (Part 3, already
  implemented). `base_footprint` still exists in the URDF (REP-105 ground-projection frame,
  parent of `base_link`) and `robot_state_publisher` still publishes it, but it's currently
  a zero-offset/identity transform from `base_link` (no separate ground-clearance modeling)
  and isn't the frame anything in this pipeline actively targets — `base_link` is the one
  real frame name to standardize on throughout.
- The EKF node (`ekf_node`), not Part 2's odometry node, broadcasts `odom → base_link`
  TF — see Part 2's note about not double-publishing this transform.
- Add both nodes to `launch/bringup_launch.py` (Part 4 item 5) and `package.xml`
  (`robot_localization`, plus `hdas_msg` for the translator's message type).

## Explicitly deferred (not in this plan)

- Depth-camera sensor plugins for `zed_link`/`left_realsense_link`/`right_realsense_link`
  — `local_costmap`'s `voxel_layer` will simply see no data until then, which is harmless
  (the `obstacle_layer`, fed by the new lidar, is enough for Nav2 to function).
- Arm/manipulation modeling and MoveIt2 — no registered BT leaf touches the arm yet.
- A real arena/world file (vs. the current empty world) and the 915 mm door validation.
- Correcting `nav_bt_stack`'s markdown docs (`bt_node_io_and_tf_tree.md`,
  `navigation_pipeline.md`, `STRUCTURE.md`) that describe frame names
  (`camera_link`, singular `arm_base_link`, `tool0`) which don't match the real Galaxea R1
  naming (`zed_link`, `left_arm_base_link`/`right_arm_base_link`) — now known-stale, but a
  documentation pass is separate from this implementation work.

## Verification

1. In the `robocup_nav` container (once Parts 1-2 land): `colcon build --packages-select
   galaxea_simulation` → `ros2 launch galaxea_simulation gazebo.launch.py` → confirm
   `ros2 topic list` shows `/livox/lidar` (PointCloud2), `/hdas/imu_chassis` (`hdas_msg/Imu`),
   and `/odom` (Odometry, `child_frame_id: base_link`), and that `/odom` is populated but
   **no** `odom → base_link` TF exists yet at this stage (that only appears once Part 7's
   EKF is running).
2. In the `nav_bt_stack` container, same host (relies on the shared `network_mode: host` +
   matching `ROS_DOMAIN_ID` already confirmed above — no extra networking setup needed):
   `colcon build --packages-select nav_bt_stack` → `ros2 launch nav_bt_stack
   bringup_launch.py use_sim_time:=true map:=<placeholder yaml>` → confirm `/scan` is being
   published (Part 6), `/imu` (`sensor_msgs/Imu`) is being published by the IMU translator
   (Part 7) off `/hdas/imu_chassis`, `/odometry/filtered` is being published and
   `ros2 run tf2_ros tf2_echo odom base_link` resolves (Part 7), Nav2 comes up, and
   `ros2 run tf2_ros tf2_echo map base_link` resolves (AMCL).
3. `ros2 run nav_bt_stack bt_main --ros-args -p tree_id:=initial_tree` — confirms the full
   chain end-to-end, matching the Phase-1 acceptance test already described in
   `STRUCTURE.md` §1 ("watch the base drive to pose 1, then pose 2"), now actually
   achievable across both containers.
