# Odometry test → SLAM+EKF+AMCL → Shim/DWB tuning → Foxglove — execution plan

## Current state (verified against the repos, not assumed)

- **`robocup_nav`** (`galaxea_simulation`), Parts 1-2 of `nav2_bringup_plan.md`: **implemented,
  uncommitted**. Confirmed by reading the actual files: `urdf/r1_sensors.urdf.xacro` adds
  `livox_frame` + `chassis_imu_link`; `launch/gazebo.launch.py` bridges
  `/livox/lidar` (`PointCloud2`) and `/_gz/imu_chassis_raw` (`sensor_msgs/Imu`); `scripts/
  imu_hdas_translator.py` republishes `/hdas/imu_chassis` as `hdas_msg/Imu`; `scripts/
  swerve_odometry.py` publishes `/odom` (raw wheel FK, **no TF broadcast** — deliberate, see
  below). So the sim now publishes what a real robot's raw drivers would: `/joint_states`,
  `/livox/lidar`, `/hdas/imu_chassis`, `/odom`.
- **`nav_bt_stack`**, Parts 3-5: implemented. `config/nav2_params.yaml` has AMCL,
  `RotationShimController` → DWB, `SmacPlannerHybrid`, costmaps; `launch/bringup_launch.py`
  brings up `nav2_bringup`'s stack + `bt_main`.
- **`nav_bt_stack`, Parts 6-7: now implemented** (see Step 0 below for the full list of what
  landed) — `pointcloud_to_laserscan`, the `hdas_msg/Imu → sensor_msgs/Imu` translator, and
  `robot_localization`'s `ekf_node` are all wired into `launch/bringup_launch.py`, publishing
  `/scan`, `/imu`, and `odom → base_link` TF once launched. `config/ekf.yaml` is no longer
  the stock upstream example — edited in place for this robot.
- Both copies of `nav2_bringup_plan.md` (`robocup_nav`'s and `nav_bt_stack`'s own, at
  `ros2_ws/src/nav2_bringup_plan.md`) now agree and are the up-to-date source of truth,
  including the resolved `base_link`-not-`base_footprint` decision below.

---

## Step 0 (prerequisite, blocks Steps 2-3) — land Parts 6-7 in `nav_bt_stack`

**Status: implemented.** Full spec in `nav2_bringup_plan.md` Parts 6-7 (now resolved and
in sync with `robocup_nav`'s copy — see the frame decision below).

- [x] `pointcloud_to_laserscan_node`, subscribing `/livox/lidar` → publishing `/scan`
      (`LaserScan`). `config/pointcloud_to_laserscan.yaml` (new): `range_max: 12.0` matching
      `nav2_params.yaml`'s `laser_max_range`, `target_frame` left empty (defaults to
      `livox_frame`, the input cloud's own frame — AMCL/costmaps already TF-transform at
      consume time). Wired into `launch/bringup_launch.py`, remapped `cloud_in` →
      `/livox/lidar`, `scan` → `/scan`.
- [x] `scripts/imu_hdas_translator.py` **in `nav_bt_stack`** (mirror image of
      `robocup_nav`'s node of the same name): subscribes `/hdas/imu_chassis`
      (`hdas_msg/Imu`), publishes `/imu` (`sensor_msgs/Imu`) — quaternion from
      roll/pitch/yaw, `angular_velocity` from `groy_x/y/z`, `linear_acceleration` from
      `acc_x/y/z`. Runs unmodified against sim or real hardware.
- [x] `config/ekf.yaml` (new, copied from the `robot_localization/params/ekf.yaml`
      stock template and edited in place, not rewritten from scratch): `odom0: /odom`
      (x, y, yaw, vx, vy, vyaw true), `imu0: /imu` (yaw, vyaw, ax, ay true),
      `two_d_mode: true`, `world_frame: odom`. `ekf_node` is the sole publisher of
      `odom → base_link` TF — `swerve_odometry.py` deliberately does not, so no second TF
      broadcaster was added. `use_control` left off (unused `control_config`/accel-limit
      params kept, commented out, as a starting point — wiring `/cmd_vel` into the EKF is a
      separate follow-up, see Step 3's unresolved accel-limit note). Rejection-threshold
      params also left commented out until Step 1 actually characterizes `/odom`'s real
      covariance instead of guessing.
- [x] `hdas_msg` vendored into `nav_bt_stack`'s own workspace
      (`ros2_ws/src/hdas_msg`, copied from `robocup_nav`'s copy — each workspace/container
      needs its own since they're separate builds, no shared workspace per the architecture
      decision).
- [x] Both Python nodes registered in `CMakeLists.txt`
      (`install(PROGRAMS scripts/imu_hdas_translator.py ...)`, same pattern
      `galaxea_simulation` uses); `pointcloud_to_laserscan`, `robot_localization`,
      `hdas_msg`, `rclpy` added to `package.xml`.
- [x] `pointcloud_to_laserscan_node`, `imu_hdas_translator.py`, and `ekf_node` wired into
      `launch/bringup_launch.py` alongside the existing `nav2_bringup` include + `bt_main`.
- [x] `Dockerfile`: added `ros-humble-pointcloud-to-laserscan` and `ros-humble-robot-localization`.

**Correction to this doc's earlier claim, found while actually verifying the build (not
assumed):** this doc previously said `robot_localization` was vendored as source under
`ros2_ws/src/robot_localization` "for the Humble lifecycle-node EKF build" — that
justification was never actually grounded in anything in `nav2_bringup_plan.md` or
elsewhere; it was an unverified assumption. Building the vendored checkout (cloned at
upstream HEAD, past the last Humble release) failed for real: its `CMakeLists.txt` links
against the `yaml-cpp::yaml-cpp` CMake target, which only exists for yaml-cpp ≥ 0.8's
namespaced export; Ubuntu Jammy's system `libyaml-cpp-dev` is 0.7.0 and only exports a
plain `yaml-cpp` target, so the build failed with "Target rl_lib links to target
yaml-cpp::yaml-cpp but the target was not found." A prebuilt `ros-humble-robot-localization`
(3.5.4) apt package exists and is what Nav2 setups normally use — switched to that instead
of patching vendored third-party source. The vendored `ros2_ws/src/robot_localization`
directory (untracked, a plain upstream clone) was deleted.

**Open decision — resolved, not silently picked:** `nav2_bringup_plan.md` Part 7 now
explicitly settles on **`base_link`**, not `base_footprint`, for the EKF's
`base_link_frame` — matching `nav2_params.yaml`'s AMCL `base_frame_id` and every other
`robot_base_frame` in the stack. Per the doc's own reasoning: `base_footprint` exists in the
URDF (REP-105 ground-projection frame) but is currently a zero-offset/identity transform
from `base_link` with no separate ground-clearance modeling, and nothing in this pipeline
actively targets it — `base_link` is the one real frame name standardized on throughout.
`config/ekf.yaml` and `swerve_odometry.py` (`robocup_nav`) both already use `base_link`
accordingly. This plan's TF-chain references below have been updated from
`odom → base_footprint` to `odom → base_link` to match.

---

## Step 1 — Test odometry accuracy (no blockers, do this first)

1. `ros2 launch galaxea_simulation gazebo.launch.py` → confirm `/joint_states`,
   `/livox/lidar`, `/hdas/imu_chassis`, `/odom` are all publishing (`ros2 topic list` /
   `ros2 topic hz`).
2. Command known `/cmd_vel` twists — pure `vx`, pure `vy`, pure `wz`, and a combined case —
   this is the exact validation already specified in `r1_gazebo_sim_plan.md` Phase 4/5 and
   `r1_odometry_sensors_localization_plan.md`'s checklist; reuse it rather than inventing a
   new test.
3. Ground truth: read Gazebo's own model pose (`gz topic -e -t /world/default/
   dynamic_pose/info`, filtered to model `r1`) and diff against `/odom`. Drive a fixed path
   (e.g. a 2 m square back to start) and record: linear drift (m of error per m traveled),
   heading drift (deg of error per 90° commanded turn). This number matters directly for
   Step 0 — it's what should set `/odom`'s covariance and the EKF's trust in it, and later
   AMCL's `alpha1-5` motion-noise params.
4. Expected at this stage: **no `odom → base_link` TF** — that only appears once the
   Step 0 EKF is running. This test reads the `/odom` topic directly, not via `tf2_echo`.

---

## Step 2 — SLAM → save map → switch to AMCL (needs Step 0)

**Phase A — SLAM (build the map):**

1. Launch: sim (`gazebo.launch.py`) + `nav_bt_stack`'s Step-0 nodes (`pointcloud_to_laserscan`,
   IMU translator, `ekf_node`) so `/scan`, `/imu`, and `odom → base_link` all exist.
2. Launch `slam_toolbox` (already in `Dockerfile`, `ros-humble-slam-toolbox`) in online-async
   mode against `/scan`; it publishes `map → odom` itself during mapping — the EKF still owns
   `odom → base_link` underneath it.
3. **Known blocker to flag, not silently work around:** the sim's only world is
   `worlds/empty.sdf` (no obstacles) — already called out as a known limitation in
   `nav2_bringup_plan.md` Part 4. SLAM against an empty world produces a degenerate map.
   Either add obstacles to the Gazebo world first, or treat this pass as a smoke test only
   (confirms the SLAM→AMCL pipeline wiring works, not real localization quality).
4. Teleop around, watch the map build (Foxglove, Step 4), then save it:
   `ros2 run nav2_map_server map_saver_cli -f <path>`.

**Phase B — switch to AMCL:**

1. Stop `slam_toolbox`. Launch `nav_bt_stack bringup_launch.py map:=<saved map.yaml>`
   (brings up AMCL per the existing `nav2_params.yaml`, plus Step 0's EKF still running
   underneath for `odom → base_link`).
2. Set initial pose (RViz/Foxglove pose-estimate tool or `/initialpose` topic).
3. Confirm the full TF chain resolves: `map → odom` (AMCL) → `odom → base_link` (EKF) →
   sensor frames. Drive around, watch the AMCL particle cloud converge and stay converged.

---

## Step 3 — Tune RotationShim + DWB (needs Step 2's localized robot + a populated map)

Start from the existing `controller_server.FollowPath` block in `nav2_params.yaml` — it's
already a deliberately-tuned `RotationShimController` wrapping DWB for the omni/swerve base,
not a stock default. Iterate by sending goals and watching behavior in Foxglove (Step 4) or
`rqt_reconfigure` for live param tweaks without relaunching:

- **Rotation-before-translate threshold** — `angular_dist_threshold` (currently `0.785` rad
  ≈ 45°). Lower if the robot rotates in place too eagerly before it should just curve toward
  the path; raise if it cuts corners aggressively.
- **Path-following tightness vs goal-seeking** — trade-off between `PathAlign.scale` /
  `PathDist.scale` (stick to the global path) and `GoalAlign.scale` / `GoalDist.scale`
  (beeline to goal). Watch actual trajectory vs. the global path in Foxglove's Path panel.
- **Goal-arrival behavior** — `xy_goal_tolerance`, `trans_stopped_velocity`, and the
  `Oscillation` critic scale if the robot hunts/oscillates near the goal instead of settling.
- **Velocity/accel limits — do not widen blindly.** `nav2_params.yaml` currently caps
  `max_vel_x/y` at 0.5 m/s and `max_vel_theta` at 1.0 rad/s. `r1_sim_plan_addendum.md` item 3
  flags an **unresolved conflict**: `mobiman` source says 0.5 m/s / 0.3 rad/s, the official
  interface doc says 1.5 m/s / 3 rad/s (accel: x ±2.5, y ±1.0, w ±1.0 m/s²/rad/s²). Resolve
  which number is real (safety-derated default vs. hardware ceiling vs. stale) before
  pushing DWB's limits toward the higher figures — if the lower ones are an intentional
  safety derate, matching the wider ones here would be a real regression, not just a tuning
  choice.
- **`SmacPlannerHybrid` tuning is a fallback, not the first move** — only touch it if the
  *global* path itself looks wrong (unnecessary loops, unrealistic reverse maneuvers since
  `minimum_turning_radius: 0.0` already reflects the omni base). The existing config is
  called out as deliberate in `nav2_bringup_plan.md` Part 3; start by trusting it.

---

## Step 4 — Visualize in Foxglove

`ros-humble-foxglove-bridge` is already in the `Dockerfile` — nothing to install.

1. Run `ros2 run foxglove_bridge foxglove_bridge` (or add it as a `Node(...)` in
   `bringup_launch.py` so it always comes up with the rest of the stack).
2. Connect Foxglove Studio/app to `ws://<host>:8765`. Both containers already run
   `network_mode: "host"` (confirmed in `nav2_bringup_plan.md`), so no extra port mapping is
   needed.
3. Useful panels for this work specifically:
   - **3D panel**: TF tree, `/scan` (LaserScan), `/livox/lidar` (PointCloud2), costmaps,
     global + local path, map.
   - **Odometry**: `/odom` (raw, Step 1) vs `/odometry/filtered` (EKF output, Step 2+) side
     by side — directly useful for judging whether the EKF fusion is actually improving on
     raw odometry.
   - **Raw Messages**: `/hdas/imu_chassis` (`hdas_msg/Imu`), AMCL particle cloud
     (`/particle_cloud`).
   - **Publish panel**: `/goal_pose` and `/initialpose`, for interactively driving Steps 2-3
     without a second RViz/CLI window.
4. Once a layout is built, export it (`config/foxglove_layout.json` in `nav_bt_stack`) so
   it's reproducible instead of rebuilt by hand each session.

---

## Suggested order

1. **Step 1** (odometry) — no blockers, start immediately.
2. **Step 4** (Foxglove) — trivial, wire in early since it helps debug the rest.
3. ~~**Step 0**~~ — done (`pointcloud_to_laserscan`, IMU translator, EKF all landed).
4. **Step 2** (SLAM → AMCL) — needs Step 0 (now satisfied), and ideally a populated Gazebo
   world (flagged above as a possible second blocker for meaningful SLAM).
5. **Step 3** (Shim/DWB tuning) — needs Step 2's localized robot on a real map.

## Open decisions carried over from the repo docs (resolve, don't silently pick)

- ~~`base_link` vs `base_footprint` for the EKF's `base_link_frame`~~ — resolved: `base_link`
  (Step 0).
- Empty Gazebo world has nothing to SLAM against (Step 2) — still open.
- Unresolved real vel/accel limits — `mobiman` (0.5 m/s / 0.3 rad/s) vs. official doc
  (1.5 m/s / 3 rad/s) — before widening DWB limits (Step 3) — still open.
- `use_control` left off in `config/ekf.yaml` (Step 0) for the same reason: wiring
  `/cmd_vel` into the EKF's prediction step means picking real acceleration limits, which
  is this same still-open item — revisit together.
- `process_noise_covariance` / `initial_estimate_covariance` in `config/ekf.yaml` are still
  the `robot_localization` stock defaults — reasonable starting point, but real tuning
  should follow Step 1's actual odometry drift measurement, not precede it.
