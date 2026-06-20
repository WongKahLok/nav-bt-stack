Part 1 — Door-crossing feature (written)
File	What it is
wait_for_door_open.hpp	[B] async leaf — polls /scan forward cone, SUCCESS when the door sector goes clear, FAILURE on timeout
voxel_check.hpp	[B] condition — samples the footprint disk in /local_costmap/costmap, FAILURE if any cell ≥ lethal
door_approach_and_cross.xml	subtree 1d: approach → wait → cross → voxel-check, wrapped in RetryUntilSuccessful (the recovery layer)
door_crossing_demo.xml	standalone test task
bt_main.cpp, CMakeLists.txt, package.xml	registered both leaves + 2 XML files; added sensor_msgs dep
A few deliberate choices worth flagging:

Subtree ports are scalars (ax/ay/ayaw, tx/ty/tyaw, gap) so they remap straight into NavToPose's existing x/y/yaw ports — no new "make-a-PoseStamped-from-numbers" node needed. In the real GPSR/HRI tree you'd instead feed door_approach_pose/door_through_pose from your KB; say the word and I'll add a pose-port variant.
Recovery is RetryUntilSuccessful, matching the docs' "recovery Fallback one level up." A proper back-up-and-retry needs a BackUp wrapper around /backup — that leaf doesn't exist yet, so the retry currently just re-approaches. I left a comment marking it.
Depth-camera gate is omitted in WaitForDoorOpen v1 (LiDAR-only); the I/O doc lists depth as a second source — easy to add as an AND gate later.
Build with: colcon build --packages-select nav_bt_stack then ros2 run nav_bt_stack bt_main --ros-args -p tree_id:=DoorCrossingDemo.

Part 2 — Integrating SmacPlannerHybrid + DWB+Shim (plan only)
The key idea: planners and controllers are not nodes in your BT. They live inside Nav2's own internal behaviour tree, behind the /navigate_to_pose action you already call. Your tree stays exactly as-is — you select them through Nav2 params, not BT XML. There is nothing to wire into your bt_main tree for basic operation.

What actually changes (3 layers)
1. planner_server params — register Smac as a named planner


planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]            # <- this STRING is the planner_id
    GridBased:
      plugin: "nav2_smac_planner::SmacPlannerHybrid"
      minimum_turning_radius: 0.0             # omni -> 0  (matches your §6 config)
      allow_unknown: true
      use_reeds_shepp: false
2. controller_server params — DWB wrapped by the Rotation Shim


controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]        # <- this STRING is the controller_id
    FollowPath:
      plugin: "nav2_rotation_shim_controller::RotationShimController"
      primary_controller: "dwb_core::DWBLocalPlanner"
      # ...DWB critics/limits nested here (your §6: min_vel_x/y -0.3, max_vel_theta 1.0,
      #    footprint tuned for the 915 mm door pass)
The shim rotates-in-place to roughly face the path, then hands off to DWB — exactly the "DWB + Shim" box in your pipeline diagram.

3. How the names reach the planner — two options:

Option A (default, simplest): Do nothing in your tree. Nav2's default bt_navigator BT calls <ComputePathToPose planner_id="GridBased"> and <FollowPath controller_id="FollowPath">. Because those IDs match your param plugin names, you're done. This is the recommended path — your NavToPose wrapper needs zero changes.

Option B (per-goal override): If you ever want different planners for different goals (e.g. a tighter planner for doorways), supply a custom Nav2 BT XML and point the goal at it via NavigateToPose.goal.behavior_tree. That means adding one optional bt_xml input port to your NavToPose wrapper that sets goal.behavior_tree. Only worth it if you need heterogeneous planning — skip for now.
-------------


1. GPSR scores with the real planner
gpsr_task.xml — added planner_id="GridBased" to InterleavedPlanner. The 3!-ordering is now scored by the same SmacPlannerHybrid that actually drives, so the chosen order reflects walls and the omni footprint, not straight lines. One-line XML change, exactly as planned.

2. Real BackUp recovery in the door subtree
New back_up.hpp — [B-thin] wrapper around Nav2 /backup, same StatefulActionNode shape as NavToPose. Registered as BackUp in bt_main.cpp.
door_approach_and_cross.xml recovery is now RetryUntilSuccessful → Fallback[ door_cross | ForceFailure(BackUp) ]. On a door-open timeout or a fouled VoxelCheck, the robot reverses 0.3 m out of the doorway, the attempt force-fails, and the retry re-approaches from the cleared pose — matching the §4 "back up 0.3 m → retry" escape. Uses nav2_msgs (already a dep), no build changes.
3. nav2_params.yaml shipped in the stack
config/nav2_params.yaml — full Nav2 config: GridBased = SmacPlannerHybrid, FollowPath = RotationShim wrapping DWB, local (obstacle+voxel+inflation) and global costmaps, AMCL, behavior_server (provides the /backup, /spin, /wait actions your wrappers call), velocity smoother. Values mirror navigation_pipeline.md §6.
CMakeLists.txt — added install(DIRECTORY config DESTINATION share/${PROJECT_NAME}), so after colcon build it lands at share/nav_bt_stack/config/nav2_params.yaml.
The wiring point that makes it all connect: the plugin names GridBased and FollowPath in the YAML are the same strings Nav2's default bt_navigator BT passes to ComputePathToPose/FollowPath, and the same planner_id your InterleavedPlanner now sends. So planner selection happens entirely in params — your task tree needs no structural change.

To use it, your bringup launch points Nav2 at the installed file:


params_file = PathJoinSubstitution([
    FindPackageShare("nav_bt_stack"), "config", "nav2_params.yaml"])
Then colcon build --packages-select nav_bt_stack. Two values to confirm against the real chassis before driving: robot_radius (0.30, must clear the 915 mm door) and the min_vel_x/y: -0.30 omni limits.

One thing I should flag honestly: I haven't compiled this — there's no built ROS 2 workspace here to run colcon against. The code follows your existing patterns and the Nav2 Humble message/plugin APIs, but the first real colcon build is the actual verification. Want me to add a launch file that brings up Nav2 with this params file, or add the bt_xml/planner_id input port to NavToPose for per-goal planner overrides?