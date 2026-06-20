#ifndef NAV_BT_STACK__FOR_EACH_POSE_HPP_
#define NAV_BT_STACK__FOR_EACH_POSE_HPP_

#include <string>
#include <vector>

#include "behaviortree_cpp_v3/decorator_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav_bt_stack
{

// Iterate a single child over a list of poses. Used by ROOM_SCAN (find) and by
// the GPSR command loop (visit_all).
//
//   mode="find"      : return SUCCESS as soon as the child SUCCEEDS on some
//                      element (stop early); FAILURE if every element fails.
//   mode="visit_all" : tick the child once per element regardless of result;
//                      SUCCESS once the list is exhausted (best-effort, so
//                      one failed command doesn't abort the rest).
//
// Before each element the current pose + index are written to output ports so
// the child can read them. If an `indices` list is supplied, current_index is
// mapped through it (so the GPSR loop can recover the ORIGINAL command index
// after the planner reordered the goals).
class ForEachPose : public BT::DecoratorNode
{
public:
  using Pose = geometry_msgs::msg::PoseStamped;

  ForEachPose(const std::string & name, const BT::NodeConfiguration & cfg)
  : BT::DecoratorNode(name, cfg) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::vector<Pose>>("poses", "list to iterate (via blackboard)"),
      BT::InputPort<std::vector<int>>("indices", "optional original-index map"),
      BT::InputPort<std::string>("mode", "find", "find | visit_all"),
      BT::OutputPort<Pose>("current_pose", "element being processed"),
      BT::OutputPort<int>("current_index", "index of current element"),
    };
  }

  BT::NodeStatus tick() override
  {
    if (!started_) {
      if (!getInput("poses", poses_) || poses_.empty()) {
        return BT::NodeStatus::FAILURE;
      }
      if (!getInput("indices", indices_)) {indices_.clear();}
      getInput("mode", mode_);
      index_ = 0;
      started_ = true;
      setCurrent();
    }

    while (index_ < poses_.size()) {
      const BT::NodeStatus child_status = child_node_->executeTick();

      if (child_status == BT::NodeStatus::RUNNING) {
        return BT::NodeStatus::RUNNING;
      }

      haltChild();  // reset child so it re-initialises for the next element

      if (mode_ == "find" && child_status == BT::NodeStatus::SUCCESS) {
        reset();
        return BT::NodeStatus::SUCCESS;
      }

      ++index_;
      if (index_ < poses_.size()) {
        setCurrent();
      }
    }

    const bool exhausted_ok = (mode_ == "visit_all");
    reset();
    return exhausted_ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  void halt() override
  {
    reset();
    haltChild();
  }

private:
  void setCurrent()
  {
    setOutput("current_pose", poses_[index_]);
    const int mapped = (index_ < indices_.size())
      ? indices_[index_] : static_cast<int>(index_);
    setOutput("current_index", mapped);
  }
  void reset()
  {
    started_ = false;
    index_ = 0;
  }

  std::vector<Pose> poses_;
  std::vector<int> indices_;
  std::string mode_{"find"};
  size_t index_{0};
  bool started_{false};
};

}  // namespace nav_bt_stack

#endif  // NAV_BT_STACK__FOR_EACH_POSE_HPP_