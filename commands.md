# pure vx (forward 0.2 m/s)

ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10

# pure vy (strafe 0.2 m/s) — this is the case that actually exercises the swerve base

ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.2, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10

# pure wz (rotate in place 0.3 rad/s)

ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}" -r 10

# combined case

ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2, y: 0.1, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}" -r 10

# in a separate terminal, bridge ground truth into ROS 2

ros2 run ros_gz_bridge parameter_bridge \
  /model/r1/pose@geometry_msgs/msg/Pose[ignition.msgs.Pose

# then record including it

ros2 bag record -s mcap -a
