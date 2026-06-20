#ifndef NAV_BT_STACK__CHECK_TARGET_FOUND_HPP_
#define NAV_BT_STACK__CHECK_TARGET_FOUND_HPP_

#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav_bt_stack/tf_utils.hpp"

namespace nav_bt_stack
{

// FEATURE 2 (part B) -- CHECK_TARGET_FOUND, FAKE perception.
//
// REAL version (Perception team contract):
//   subscribe to detections (e.g. vision_msgs/Detection3DArray) -> match the
//   requested label / track id -> transform the hit into `map` -> setOutput
//   "found_pose". The rest of ROOM_SCAN doesn't change.
//
// FAKE version (now): a ground-truth target table comes from the ROS param
// `fake_targets` (list of "name:x:y"). The check SUCCEEDS when the robot is
// within `detection_range` of the named target. This makes ROOM_SCAN actually
// terminate in sim once a viewpoint gets close enough, exercising the loop.
class CheckTargetFound : public BT::ConditionNode
{
public:
  using Pose = geometry_msgs::msg::PoseStamped;

  CheckTargetFound(
    const std::string & name, const BT::NodeConfiguration & cfg,
    const rclcpp::Node::SharedPtr & node)
  : BT::ConditionNode(name, cfg), node_(node), robot_pose_(node)
  {
    detection_range_ = node_->has_parameter("detection_range")
      ? node_->get_parameter("detection_range").as_double()
      : node_->declare_parameter<double>("detection_range", 1.5);

    const std::vector<std::string> entries = node_->has_parameter("fake_targets")
      ? node_->get_parameter("fake_targets").as_string_array()
      : node_->declare_parameter<std::vector<std::string>>(
          "fake_targets", {"cup:6.0:1.0", "person:1.0:3.0", "desk:7.0:6.0"});

    for (const auto & e : entries) {
      std::stringstream ss(e);
      std::string tok, xs, ys;
      if (std::getline(ss, tok, ':') && std::getline(ss, xs, ':') && std::getline(ss, ys, ':')) {
        Pose p;
        p.header.frame_id = "map";
        p.pose.position.x = std::stod(xs);
        p.pose.position.y = std::stod(ys);
        p.pose.orientation.w = 1.0;
        fake_targets_[tok] = p;
      }
    }
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("target", "label / class to look for"),
      BT::OutputPort<Pose>("found_pose", "pose of the target if found"),
    };
  }

  BT::NodeStatus tick() override
  {
    std::string target;
    getInput("target", target);

    const auto it = fake_targets_.find(target);
    if (it == fake_targets_.end()) {
      return BT::NodeStatus::FAILURE;  // target not in the (fake) world
    }

    Pose robot;
    if (!robot_pose_.get(robot)) {
      return BT::NodeStatus::FAILURE;  // no TF yet
    }

    const double d = poseDistance(robot, it->second);
    if (d <= detection_range_) {
      Pose found = it->second;
      found.header.stamp = node_->now();
      setOutput("found_pose", found);
      RCLCPP_INFO(
        node_->get_logger(),
        "[CheckTargetFound] FOUND '%s' at (%.1f, %.1f), range %.2fm",
        target.c_str(), found.pose.position.x, found.pose.position.y, d);
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }

private:
  rclcpp::Node::SharedPtr node_;
  RobotPose robot_pose_;
  double detection_range_{1.5};
  std::map<std::string, Pose> fake_targets_;
};

}  // namespace nav_bt_stack

#endif  // NAV_BT_STACK__CHECK_TARGET_FOUND_HPP_