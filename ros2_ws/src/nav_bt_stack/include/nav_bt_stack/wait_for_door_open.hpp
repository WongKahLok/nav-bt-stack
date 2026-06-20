#ifndef NAV_BT_STACK__WAIT_FOR_DOOR_OPEN_HPP_
#define NAV_BT_STACK__WAIT_FOR_DOOR_OPEN_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace nav_bt_stack
{

// ---------------------------------------------------------------------------
// WaitForDoorOpen  --  tag [B] (real-logic leaf)
//
// DOOR_APPROACH_AND_CROSS step 2. The robot is already parked ~1 m in front of
// the (closed) door, facing it. We poll the LiDAR forward cone and SUCCEED the
// instant that cone goes clear -- i.e. the door has opened.
//
// Logic (per tick, async StatefulActionNode):
//   * take the forward sector [bearing - half, bearing + half] of /scan
//   * a beam is "clear" if its range > gap_threshold OR is non-finite (no
//     return = open space)
//   * if the clear fraction of the sector >= clear_ratio -> SUCCESS (door open)
//   * else RUNNING, until timeout_s elapses -> FAILURE (recovery handles it)
//
// REAL sensor, FAKE nothing -- this works against a live /scan immediately.
// Depth-camera confirmation (camera_depth_optical_frame) is intentionally left
// out for v1; add it as a second gate here later if LiDAR alone proves noisy.
// ---------------------------------------------------------------------------
class WaitForDoorOpen : public BT::StatefulActionNode
{
public:
  using Scan = sensor_msgs::msg::LaserScan;

  WaitForDoorOpen(
    const std::string & name, const BT::NodeConfiguration & cfg,
    const rclcpp::Node::SharedPtr & node)
  : BT::StatefulActionNode(name, cfg), node_(node) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("scan_topic", "/scan", "LaserScan topic"),
      BT::InputPort<double>("gap_threshold", 1.5, "range [m] above which a beam reads 'open'"),
      BT::InputPort<double>("bearing", 0.0, "centre of the door sector [rad, base_scan]"),
      BT::InputPort<double>("cone_half_angle", 0.35, "half-width of the door sector [rad]"),
      BT::InputPort<double>("clear_ratio", 0.7, "fraction of sector beams that must be clear"),
      BT::InputPort<double>("timeout_s", 30.0, "give up after this long -> FAILURE"),
    };
  }

  BT::NodeStatus onStart() override
  {
    std::string topic = "/scan";
    getInput("scan_topic", topic);
    getInput("gap_threshold", gap_threshold_);
    getInput("bearing", bearing_);
    getInput("cone_half_angle", cone_half_angle_);
    getInput("clear_ratio", clear_ratio_);
    getInput("timeout_s", timeout_s_);

    // (re)subscribe if the topic changed since last run
    if (!sub_ || topic != topic_) {
      topic_ = topic;
      sub_ = node_->create_subscription<Scan>(
        topic_, rclcpp::SensorDataQoS(),
        [this](Scan::ConstSharedPtr msg) {last_scan_ = msg;});
    }
    last_scan_.reset();
    start_time_ = node_->now();
    RCLCPP_INFO(node_->get_logger(), "[WaitForDoorOpen] watching '%s' for door to open", topic_.c_str());
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    if ((node_->now() - start_time_).seconds() > timeout_s_) {
      RCLCPP_WARN(node_->get_logger(), "[WaitForDoorOpen] timeout -> door still closed");
      return BT::NodeStatus::FAILURE;
    }
    if (!last_scan_) {
      return BT::NodeStatus::RUNNING;  // no scan yet
    }

    const Scan & s = *last_scan_;
    int total = 0, clear = 0;
    for (size_t i = 0; i < s.ranges.size(); ++i) {
      const double a = s.angle_min + static_cast<double>(i) * s.angle_increment;
      if (a < bearing_ - cone_half_angle_ || a > bearing_ + cone_half_angle_) {
        continue;  // outside the door sector
      }
      ++total;
      const float r = s.ranges[i];
      if (!std::isfinite(r) || r > gap_threshold_) {++clear;}  // no return or far = open
    }

    if (total > 0 && static_cast<double>(clear) / total >= clear_ratio_) {
      RCLCPP_INFO(
        node_->get_logger(), "[WaitForDoorOpen] door OPEN (%d/%d beams clear)", clear, total);
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override {}

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<Scan>::SharedPtr sub_;
  Scan::ConstSharedPtr last_scan_;
  std::string topic_;

  double gap_threshold_{1.5};
  double bearing_{0.0};
  double cone_half_angle_{0.35};
  double clear_ratio_{0.7};
  double timeout_s_{30.0};
  rclcpp::Time start_time_;
};

}  // namespace nav_bt_stack

#endif  // NAV_BT_STACK__WAIT_FOR_DOOR_OPEN_HPP_
