#ifndef NAV_BT_STACK__GPSR_MOCKS_HPP_
#define NAV_BT_STACK__GPSR_MOCKS_HPP_

#include <string>
#include <vector>

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav_bt_stack
{

// ===========================================================================
// MOCKS -- stand in for other teams until their interfaces land. Each is
// clearly fake and easy to delete once the real producer exists.
// ===========================================================================

// FAKE: the HRI command parser. Emits 3 GPSR commands as parallel arrays so the
// planner + executor have real data. Replace with the HRI module's output.
//
// Command i = (goal pose, kind, target label, room, located?)
//   located=1 -> coordinates known, navigate directly
//   located=0 -> unknown spot in `room` -> must ROOM_SCAN
class MockGpsrCommands : public BT::SyncActionNode
{
public:
  using Pose = geometry_msgs::msg::PoseStamped;

  MockGpsrCommands(
    const std::string & n, const BT::NodeConfiguration & c,
    const rclcpp::Node::SharedPtr & node)
  : BT::SyncActionNode(n, c), node_(node) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<std::vector<Pose>>("goals"),
      BT::OutputPort<std::vector<std::string>>("kinds"),     // location | person | object
      BT::OutputPort<std::vector<std::string>>("targets"),   // label to look for
      BT::OutputPort<std::vector<std::string>>("rooms"),     // where to scan if unlocated
      BT::OutputPort<std::vector<int>>("located"),           // 1 = coords known
    };
  }

  BT::NodeStatus tick() override
  {
    auto mk = [&](double x, double y) {
        Pose p;
        p.header.frame_id = "map";
        p.header.stamp = node_->now();
        p.pose.position.x = x;
        p.pose.position.y = y;
        p.pose.orientation.w = 1.0;
        return p;
      };

    // command 0: pick the cup (kitchen, located)
    // command 1: find the person (living room, NOT located -> ROOM_SCAN)
    // command 2: go to the desk (office, located)
    setOutput<std::vector<Pose>>("goals", {mk(6.0, 1.0), mk(2.5, 2.0), mk(7.0, 6.0)});
    setOutput<std::vector<std::string>>("kinds", {"object", "person", "location"});
    setOutput<std::vector<std::string>>("targets", {"cup", "person", "desk"});
    setOutput<std::vector<std::string>>("rooms", {"kitchen", "living_room", "office"});
    setOutput<std::vector<int>>("located", {1, 0, 1});
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
};

// Read a string out of a parallel array at `index` -> output port `value`.
// Used to pull this command's target label or room name onto the blackboard.
class CommandField : public BT::SyncActionNode
{
public:
  CommandField(const std::string & n, const BT::NodeConfiguration & c)
  : BT::SyncActionNode(n, c) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::vector<std::string>>("array"),
      BT::InputPort<int>("index"),
      BT::OutputPort<std::string>("value"),
    };
  }

  BT::NodeStatus tick() override
  {
    std::vector<std::string> arr;
    int idx = 0;
    if (!getInput("array", arr) || !getInput("index", idx)) {return BT::NodeStatus::FAILURE;}
    if (idx < 0 || idx >= static_cast<int>(arr.size())) {return BT::NodeStatus::FAILURE;}
    setOutput("value", arr[idx]);
    return BT::NodeStatus::SUCCESS;
  }
};

// Condition: kinds[index] == is ?
class CommandKindIs : public BT::ConditionNode
{
public:
  CommandKindIs(const std::string & n, const BT::NodeConfiguration & c)
  : BT::ConditionNode(n, c) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::vector<std::string>>("kinds"),
      BT::InputPort<int>("index"),
      BT::InputPort<std::string>("is", "location | person | object"),
    };
  }

  BT::NodeStatus tick() override
  {
    std::vector<std::string> kinds;
    int idx = 0;
    std::string want;
    if (!getInput("kinds", kinds) || !getInput("index", idx) || !getInput("is", want)) {
      return BT::NodeStatus::FAILURE;
    }
    if (idx < 0 || idx >= static_cast<int>(kinds.size())) {return BT::NodeStatus::FAILURE;}
    return (kinds[idx] == want) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

// Condition: located[index] == 1 ?
class CommandLocated : public BT::ConditionNode
{
public:
  CommandLocated(const std::string & n, const BT::NodeConfiguration & c)
  : BT::ConditionNode(n, c) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::vector<int>>("located"),
      BT::InputPort<int>("index"),
    };
  }

  BT::NodeStatus tick() override
  {
    std::vector<int> loc;
    int idx = 0;
    if (!getInput("located", loc) || !getInput("index", idx)) {return BT::NodeStatus::FAILURE;}
    if (idx < 0 || idx >= static_cast<int>(loc.size())) {return BT::NodeStatus::FAILURE;}
    return (loc[idx] != 0) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

}  // namespace nav_bt_stack

#endif  // NAV_BT_STACK__GPSR_MOCKS_HPP_