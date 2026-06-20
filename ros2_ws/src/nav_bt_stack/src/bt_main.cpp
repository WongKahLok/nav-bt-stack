#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "nav_bt_stack/nav_to_pose.hpp"
#include "nav_bt_stack/back_up.hpp"
#include "nav_bt_stack/interleaved_planner.hpp"
#include "nav_bt_stack/for_each_pose.hpp"
#include "nav_bt_stack/get_viewpoints.hpp"
#include "nav_bt_stack/check_target_found.hpp"
#include "nav_bt_stack/wait_for_door_open.hpp"
#include "nav_bt_stack/voxel_check.hpp"
#include "nav_bt_stack/gpsr_mocks.hpp"

using namespace std::chrono_literals;

// Helper: register a leaf that needs the rclcpp node in its constructor.
template<typename T>
static void registerRosLeaf(
  BT::BehaviorTreeFactory & factory, const std::string & tag,
  const rclcpp::Node::SharedPtr & node)
{
  BT::NodeBuilder builder =
    [node](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<T>(name, config, node);
    };
  factory.registerBuilder<T>(tag, builder);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("nav_bt_stack");

  const std::string share = ament_index_cpp::get_package_share_directory("nav_bt_stack");
  // which tree to run; see bt_xml/tasks for the available IDs
  const std::string tree_id = node->declare_parameter<std::string>("tree_id", "GpsrTask");
  const double tick_hz = node->declare_parameter<double>("tick_hz", 20.0);

  BT::BehaviorTreeFactory factory;

  // --- (A) register leaves: XML tag string -> C++ class -------------------
  // ROS leaves (need the node):
  registerRosLeaf<nav_bt_stack::NavToPose>(factory, "NavToPose", node);
  registerRosLeaf<nav_bt_stack::BackUp>(factory, "BackUp", node);
  registerRosLeaf<nav_bt_stack::InterleavedPlanner>(factory, "InterleavedPlanner", node);
  registerRosLeaf<nav_bt_stack::GetViewpoints>(factory, "GetViewpoints", node);
  registerRosLeaf<nav_bt_stack::CheckTargetFound>(factory, "CheckTargetFound", node);
  registerRosLeaf<nav_bt_stack::WaitForDoorOpen>(factory, "WaitForDoorOpen", node);
  registerRosLeaf<nav_bt_stack::VoxelCheck>(factory, "VoxelCheck", node);
  registerRosLeaf<nav_bt_stack::MockGpsrCommands>(factory, "MockGpsrCommands", node);
  // pure-logic leaves (no ROS needed):
  factory.registerNodeType<nav_bt_stack::ForEachPose>("ForEachPose");
  factory.registerNodeType<nav_bt_stack::CommandField>("CommandField");
  factory.registerNodeType<nav_bt_stack::CommandKindIs>("CommandKindIs");
  factory.registerNodeType<nav_bt_stack::CommandLocated>("CommandLocated");

  // --- (B) register every tree file (subtrees first, then tasks) ----------
  factory.registerBehaviorTreeFromFile(share + "/bt_xml/subtrees/room_scan.xml");
  factory.registerBehaviorTreeFromFile(share + "/bt_xml/subtrees/door_approach_and_cross.xml");
  factory.registerBehaviorTreeFromFile(share + "/bt_xml/tasks/interleaved_demo.xml");
  factory.registerBehaviorTreeFromFile(share + "/bt_xml/tasks/room_scan_demo.xml");
  factory.registerBehaviorTreeFromFile(share + "/bt_xml/tasks/door_crossing_demo.xml");
  factory.registerBehaviorTreeFromFile(share + "/bt_xml/tasks/gpsr_task.xml");

  // --- (C) build the requested tree (resolves <SubTree> refs by ID) -------
  RCLCPP_INFO(node->get_logger(), "Building tree '%s'", tree_id.c_str());
  auto tree = factory.createTree(tree_id);

  rclcpp::Rate rate(tick_hz);
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
    status = tree.tickRoot();
    rclcpp::spin_some(node);
    rate.sleep();
  }

  RCLCPP_INFO(
    node->get_logger(), "Tree '%s' finished: %s", tree_id.c_str(),
    status == BT::NodeStatus::SUCCESS ? "SUCCESS" : "FAILURE");

  rclcpp::shutdown();
  return 0;
}