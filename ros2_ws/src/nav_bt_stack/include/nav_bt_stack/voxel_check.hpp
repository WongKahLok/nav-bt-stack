#ifndef NAV_BT_STACK__VOXEL_CHECK_HPP_
#define NAV_BT_STACK__VOXEL_CHECK_HPP_

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/time.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace nav_bt_stack
{

// ---------------------------------------------------------------------------
// VoxelCheck  --  tag [B] (condition leaf)
//
// DOOR_APPROACH_AND_CROSS step 4. After crossing, assert the robot footprint is
// actually clear in the (3D-fed) local costmap before the Sequence declares the
// crossing done. Ghost inflation from the door frame, the swinging door, or a
// person in the doorway can leave the footprint "inside" an obstacle, which
// breaks the next planning call. If not clear -> FAILURE, and the recovery
// Fallback one level up handles it (back up + retry).
//
// Reads the latest /local_costmap/costmap (nav2_msgs/Costmap) and looks up the
// robot pose in the costmap's own frame via TF (handles odom- or map-rooted
// local costmaps). Samples every cell within footprint_radius of the robot; if
// any cell >= lethal_threshold the footprint is fouled.
// ---------------------------------------------------------------------------
class VoxelCheck : public BT::ConditionNode
{
public:
  using Costmap = nav2_msgs::msg::Costmap;

  VoxelCheck(
    const std::string & name, const BT::NodeConfiguration & cfg,
    const rclcpp::Node::SharedPtr & node)
  : BT::ConditionNode(name, cfg), node_(node)
  {
    buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
    sub_ = node_->create_subscription<Costmap>(
      "/local_costmap/costmap", rclcpp::QoS(1).transient_local(),
      [this](Costmap::ConstSharedPtr msg) {last_map_ = msg;});
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("lethal_threshold", 254, "cell cost >= this counts as blocked"),
      BT::InputPort<double>("footprint_radius", 0.30, "robot radius to sample [m]"),
      BT::InputPort<std::string>("base_frame", "base_link", "robot frame to locate"),
    };
  }

  BT::NodeStatus tick() override
  {
    int lethal = 254;
    double radius = 0.30;
    std::string base = "base_link";
    getInput("lethal_threshold", lethal);
    getInput("footprint_radius", radius);
    getInput("base_frame", base);

    if (!last_map_) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "[VoxelCheck] no /local_costmap/costmap yet");
      return BT::NodeStatus::FAILURE;
    }
    const Costmap & m = *last_map_;

    // robot pose in the costmap's frame
    double rx, ry;
    try {
      const auto tf = buffer_->lookupTransform(
        m.header.frame_id, base, tf2::TimePointZero);
      rx = tf.transform.translation.x;
      ry = tf.transform.translation.y;
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "[VoxelCheck] TF %s->%s unavailable: %s",
        m.header.frame_id.c_str(), base.c_str(), e.what());
      return BT::NodeStatus::FAILURE;
    }

    const double res = m.metadata.resolution;
    const double ox = m.metadata.origin.position.x;
    const double oy = m.metadata.origin.position.y;
    const int sx = static_cast<int>(m.metadata.size_x);
    const int sy = static_cast<int>(m.metadata.size_y);
    if (res <= 0.0 || sx <= 0 || sy <= 0) {return BT::NodeStatus::FAILURE;}

    const int cx = static_cast<int>((rx - ox) / res);
    const int cy = static_cast<int>((ry - oy) / res);
    const int rad_cells = static_cast<int>(std::ceil(radius / res));

    int worst = 0;
    for (int dy = -rad_cells; dy <= rad_cells; ++dy) {
      for (int dx = -rad_cells; dx <= rad_cells; ++dx) {
        if (dx * dx + dy * dy > rad_cells * rad_cells) {continue;}  // disk, not square
        const int gx = cx + dx, gy = cy + dy;
        if (gx < 0 || gy < 0 || gx >= sx || gy >= sy) {continue;}  // off the local map
        const int cost = m.data[static_cast<size_t>(gy) * sx + gx];
        worst = std::max(worst, cost);
        if (cost >= lethal) {
          RCLCPP_WARN(
            node_->get_logger(),
            "[VoxelCheck] footprint FOULED: cell (%d,%d) cost=%d >= %d", gx, gy, cost, lethal);
          return BT::NodeStatus::FAILURE;
        }
      }
    }
    RCLCPP_INFO(node_->get_logger(), "[VoxelCheck] footprint clear (worst cost=%d)", worst);
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<Costmap>::SharedPtr sub_;
  Costmap::ConstSharedPtr last_map_;
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
};

}  // namespace nav_bt_stack

#endif  // NAV_BT_STACK__VOXEL_CHECK_HPP_
