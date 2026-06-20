#ifndef NAV_BT_STACK__TF_UTILS_HPP_
#define NAV_BT_STACK__TF_UTILS_HPP_

#include <cmath>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/time.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace nav_bt_stack
{

// Owns a TF buffer/listener and returns the robot's current pose in
// `global_frame` (default "map"). Returns false if TF isn't ready yet.
// Several leaves need "where am I now"; give each its own instance.
class RobotPose
{
public:
  explicit RobotPose(
    const rclcpp::Node::SharedPtr & node,
    std::string global_frame = "map",
    std::string base_frame = "base_link")
  : node_(node), global_frame_(std::move(global_frame)), base_frame_(std::move(base_frame))
  {
    buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
  }

  bool get(geometry_msgs::msg::PoseStamped & out)
  {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = buffer_->lookupTransform(global_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "[RobotPose] TF %s->%s unavailable: %s",
        global_frame_.c_str(), base_frame_.c_str(), e.what());
      return false;
    }
    out.header.frame_id = global_frame_;
    out.header.stamp = node_->now();
    out.pose.position.x = tf.transform.translation.x;
    out.pose.position.y = tf.transform.translation.y;
    out.pose.position.z = tf.transform.translation.z;
    out.pose.orientation = tf.transform.rotation;
    return true;
  }

  const std::string & global_frame() const {return global_frame_;}

private:
  rclcpp::Node::SharedPtr node_;
  std::string global_frame_;
  std::string base_frame_;
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
};

inline double poseDistance(
  const geometry_msgs::msg::PoseStamped & a,
  const geometry_msgs::msg::PoseStamped & b)
{
  return std::hypot(
    a.pose.position.x - b.pose.position.x,
    a.pose.position.y - b.pose.position.y);
}

}  // namespace nav_bt_stack

#endif  // NAV_BT_STACK__TF_UTILS_HPP_