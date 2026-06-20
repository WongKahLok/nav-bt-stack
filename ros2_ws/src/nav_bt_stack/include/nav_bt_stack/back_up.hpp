#ifndef NAV_BT_STACK__BACK_UP_HPP_
#define NAV_BT_STACK__BACK_UP_HPP_

#include <chrono>
#include <future>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/back_up.hpp"

namespace nav_bt_stack
{

// ---------------------------------------------------------------------------
// BackUp  --  tag [B-thin]
//
// Thin wrapper around the Nav2 `backup` recovery action. Drives the base
// straight backward `distance` metres at `speed`. Used as the recovery step in
// DOOR_APPROACH_AND_CROSS: if the crossing sequence fails (door-open timeout or
// a fouled footprint from VoxelCheck), reverse out of the doorway before the
// retry re-approaches.
//
// Same StatefulActionNode shape as NavToPose: onStart() sends the goal,
// onRunning() polls without blocking, onHalted() cancels.
// ---------------------------------------------------------------------------
class BackUp : public BT::StatefulActionNode
{
public:
  using ActionT = nav2_msgs::action::BackUp;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActionT>;

  BackUp(
    const std::string & name, const BT::NodeConfiguration & config,
    const rclcpp::Node::SharedPtr & node)
  : BT::StatefulActionNode(name, config), node_(node)
  {
    client_ = rclcpp_action::create_client<ActionT>(node_, "backup");
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("distance", 0.30, "reverse distance [m]"),
      BT::InputPort<double>("speed", 0.15, "reverse speed [m/s, positive]"),
      BT::InputPort<double>("time_allowance", 10.0, "abort if not done within [s]"),
    };
  }

  BT::NodeStatus onStart() override
  {
    if (!client_->wait_for_action_server(std::chrono::seconds(2))) {
      RCLCPP_ERROR(node_->get_logger(), "[BackUp] action server 'backup' not available");
      return BT::NodeStatus::FAILURE;
    }

    double distance = 0.30, speed = 0.15, allowance = 10.0;
    getInput("distance", distance);
    getInput("speed", speed);
    getInput("time_allowance", allowance);

    ActionT::Goal goal;
    goal.target.x = distance;               // BackUp drives along -x by target.x metres
    goal.speed = static_cast<float>(speed);
    goal.time_allowance = rclcpp::Duration::from_seconds(allowance);

    done_ = false;
    accepted_ = false;
    result_code_ = rclcpp_action::ResultCode::UNKNOWN;

    rclcpp_action::Client<ActionT>::SendGoalOptions opts;
    opts.result_callback =
      [this](const GoalHandle::WrappedResult & result) {
        done_ = true;
        result_code_ = result.code;
      };
    future_goal_handle_ = client_->async_send_goal(goal, opts);

    RCLCPP_INFO(node_->get_logger(), "[BackUp] reversing %.2f m at %.2f m/s", distance, speed);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    if (!accepted_) {
      if (future_goal_handle_.wait_for(std::chrono::seconds(0)) ==
        std::future_status::ready)
      {
        goal_handle_ = future_goal_handle_.get();
        if (!goal_handle_) {
          RCLCPP_ERROR(node_->get_logger(), "[BackUp] goal rejected");
          return BT::NodeStatus::FAILURE;
        }
        accepted_ = true;
      } else {
        return BT::NodeStatus::RUNNING;
      }
    }
    if (done_) {
      if (result_code_ == rclcpp_action::ResultCode::SUCCEEDED) {
        return BT::NodeStatus::SUCCESS;
      }
      RCLCPP_WARN(node_->get_logger(), "[BackUp] backup did not succeed");
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    if (accepted_ && goal_handle_) {
      client_->async_cancel_goal(goal_handle_);
      RCLCPP_INFO(node_->get_logger(), "[BackUp] halted -> cancelling goal");
    }
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<ActionT>::SharedPtr client_;
  std::shared_future<GoalHandle::SharedPtr> future_goal_handle_;
  GoalHandle::SharedPtr goal_handle_;

  bool done_{false};
  bool accepted_{false};
  rclcpp_action::ResultCode result_code_{rclcpp_action::ResultCode::UNKNOWN};
};

}  // namespace nav_bt_stack

#endif  // NAV_BT_STACK__BACK_UP_HPP_
