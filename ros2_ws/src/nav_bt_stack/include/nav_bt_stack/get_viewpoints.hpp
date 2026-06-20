#ifndef NAV_BT_STACK__GET_VIEWPOINTS_HPP_
#define NAV_BT_STACK__GET_VIEWPOINTS_HPP_

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav_bt_stack
{

// FEATURE 2 (part A) -- generate N coverage viewpoints inside a room's
// bounding box, laid out serpentine, each yaw pointing at the room centre so
// perception scans inward.
//
// FAKED: lookupRoom() is a hardcoded table standing in for the real map /
// knowledge base. Replace it with a KB or map-server query; everything else
// stays.
class GetViewpoints : public BT::SyncActionNode
{
public:
  using Pose = geometry_msgs::msg::PoseStamped;

  GetViewpoints(
    const std::string & name, const BT::NodeConfiguration & cfg,
    const rclcpp::Node::SharedPtr & node)
  : BT::SyncActionNode(name, cfg), node_(node) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("room", "room name (looked up to bounds)"),
      BT::InputPort<int>("num_viewpoints", 6, "coverage poses (4-6 typical)"),
      BT::InputPort<std::string>("frame_id", "map", "frame of the output poses"),
      BT::OutputPort<std::vector<Pose>>("viewpoints", "coverage poses"),
    };
  }

  BT::NodeStatus tick() override
  {
    std::string room, frame = "map";
    int n = 6;
    getInput("room", room);
    getInput("num_viewpoints", n);
    getInput("frame_id", frame);
    n = std::max(2, n);

    double minx, miny, maxx, maxy;
    if (!lookupRoom(room, minx, miny, maxx, maxy)) {
      RCLCPP_ERROR(node_->get_logger(), "[GetViewpoints] unknown room '%s'", room.c_str());
      return BT::NodeStatus::FAILURE;
    }

    const double cx = 0.5 * (minx + maxx), cy = 0.5 * (miny + maxy);
    const double mx = 0.1 * (maxx - minx), my = 0.1 * (maxy - miny);  // wall margin
    minx += mx; maxx -= mx; miny += my; maxy -= my;

    const int cols = static_cast<int>(std::ceil(std::sqrt(static_cast<double>(n))));
    const int rows = static_cast<int>(std::ceil(static_cast<double>(n) / cols));

    std::vector<Pose> vps;
    int made = 0;
    for (int r = 0; r < rows && made < n; ++r) {
      for (int c = 0; c < cols && made < n; ++c) {
        const int cc = (r % 2 == 0) ? c : (cols - 1 - c);  // serpentine
        const double x = (cols > 1) ? minx + (maxx - minx) * cc / (cols - 1) : cx;
        const double y = (rows > 1) ? miny + (maxy - miny) * r / (rows - 1) : cy;

        Pose p;
        p.header.frame_id = frame;
        p.header.stamp = node_->now();
        p.pose.position.x = x;
        p.pose.position.y = y;
        const double yaw = std::atan2(cy - y, cx - x);  // look toward room centre
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        p.pose.orientation = tf2::toMsg(q);
        vps.push_back(p);
        ++made;
      }
    }

    setOutput("viewpoints", vps);
    RCLCPP_INFO(
      node_->get_logger(), "[GetViewpoints] %zu viewpoints in '%s'", vps.size(), room.c_str());
    return BT::NodeStatus::SUCCESS;
  }

private:
  // FAKE knowledge base. Replace with a real map/KB query.
  bool lookupRoom(
    const std::string & room, double & minx, double & miny,
    double & maxx, double & maxy) const
  {
    if (room == "living_room") {minx = 0; miny = 0; maxx = 5; maxy = 4; return true;}
    if (room == "kitchen") {minx = 5; miny = 0; maxx = 9; maxy = 4; return true;}
    if (room == "bedroom") {minx = 0; miny = 4; maxx = 4; maxy = 8; return true;}
    if (room == "office") {minx = 4; miny = 4; maxx = 9; maxy = 8; return true;}
    if (!room.empty()) {minx = 0; miny = 0; maxx = 4; maxy = 4; return true;}  // generic
    return false;
  }

  rclcpp::Node::SharedPtr node_;
};

}  // namespace nav_bt_stack

#endif  // NAV_BT_STACK__GET_VIEWPOINTS_HPP_