#ifndef NAV_BT_STACK__INTERLEAVED_PLANNER_HPP_
#define NAV_BT_STACK__INTERLEAVED_PLANNER_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav_bt_stack/tf_utils.hpp"

namespace nav_bt_stack
{

// ===========================================================================
// FEATURE 1 -- INTERLEAVED path-cost planner (GPSR)
//
// When all command goal-poses are received at once, find the order that
// minimises total travel. Scoring uses the REAL Nav2 global planner
// (compute_path_to_pose) so cost reflects walls/obstacles, not straight lines.
//
// How it works:
//   1. Read N goal poses + the robot's current pose (= START).
//   2. Build a cost matrix: cost[from][to] for START->each and each->each.
//      Each entry is one compute_path_to_pose query; we run them one per tick
//      (the action is async, never block the tree).
//   3. Enumerate all N! orderings, total each, pick the minimum.
//   4. Output the ordered goals + original indices, plus naive vs best cost so
//      the tree can decide whether the interleaved bonus is "demonstrably
//      time-saving".
//
// Fallback: if use_planner=false or the planner action isn't up, costs are
// straight-line distances (lets you test offline). N is meant to be small
// (GPSR = 3); N! grows fast, so it warns above 6.
// ===========================================================================
class InterleavedPlanner : public BT::StatefulActionNode
{
public:
  using ComputePath = nav2_msgs::action::ComputePathToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ComputePath>;
  using Pose = geometry_msgs::msg::PoseStamped;

  InterleavedPlanner(
    const std::string & name, const BT::NodeConfiguration & cfg,
    const rclcpp::Node::SharedPtr & node)
  : BT::StatefulActionNode(name, cfg), node_(node), robot_pose_(node)
  {
    client_ = rclcpp_action::create_client<ComputePath>(node_, "compute_path_to_pose");
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::vector<Pose>>("goals", "command goal-poses (via blackboard)"),
      BT::InputPort<bool>("use_planner", true, "true: Nav2 path cost; false: straight-line"),
      BT::InputPort<std::string>("planner_id", "", "Nav2 planner plugin id ('' = default)"),
      BT::InputPort<double>("saving_margin", 0.02, "min fractional gain to flag time_saving"),
      BT::OutputPort<std::vector<Pose>>("ordered_goals", "goals in optimal order"),
      BT::OutputPort<std::vector<int>>("ordered_indices", "original indices in optimal order"),
      BT::OutputPort<double>("naive_cost", "cost of the as-received order [m]"),
      BT::OutputPort<double>("best_cost", "cost of the optimal order [m]"),
      BT::OutputPort<bool>("time_saving", "true if optimal beats naive by >= margin"),
    };
  }

  BT::NodeStatus onStart() override
  {
    if (!getInput("goals", goals_) || goals_.empty()) {
      RCLCPP_ERROR(node_->get_logger(), "[InterleavedPlanner] no goals provided");
      return BT::NodeStatus::FAILURE;
    }
    getInput("use_planner", use_planner_);
    getInput("planner_id", planner_id_);
    getInput("saving_margin", saving_margin_);

    if (goals_.size() > 6) {
      RCLCPP_WARN(
        node_->get_logger(),
        "[InterleavedPlanner] %zu goals -> %zu! orderings; this is brute force",
        goals_.size(), goals_.size());
    }

    if (!robot_pose_.get(start_)) {
      RCLCPP_ERROR(node_->get_logger(), "[InterleavedPlanner] no robot start pose from TF");
      return BT::NodeStatus::FAILURE;
    }

    // queries we need: START->i for all i, and i->j for all i!=j  (from=-1 = START)
    pairs_.clear();
    const int n = static_cast<int>(goals_.size());
    for (int i = 0; i < n; ++i) {pairs_.emplace_back(-1, i);}
    for (int i = 0; i < n; ++i) {
      for (int j = 0; j < n; ++j) {
        if (i != j) {pairs_.emplace_back(i, j);}
      }
    }
    pair_idx_ = 0;
    costs_.clear();
    inflight_ = false;
    accepted_ = false;
    result_ready_ = false;

    if (use_planner_ && !client_->wait_for_action_server(std::chrono::seconds(2))) {
      RCLCPP_WARN(
        node_->get_logger(),
        "[InterleavedPlanner] compute_path_to_pose unavailable -> straight-line fallback");
      use_planner_ = false;
    }

    if (!use_planner_) {
      for (const auto & p : pairs_) {costs_[p] = euclid(p);}
      finish();
      return BT::NodeStatus::SUCCESS;  // computed synchronously
    }
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    if (pair_idx_ >= pairs_.size()) {
      finish();
      return BT::NodeStatus::SUCCESS;
    }
    if (!inflight_) {
      sendQuery(pairs_[pair_idx_]);
      return BT::NodeStatus::RUNNING;
    }
    if (!accepted_) {
      if (future_gh_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        gh_ = future_gh_.get();
        if (!gh_) {                              // rejected -> straight-line for this pair
          costs_[pairs_[pair_idx_]] = euclid(pairs_[pair_idx_]);
          advance();
        } else {
          accepted_ = true;
        }
      }
      return BT::NodeStatus::RUNNING;
    }
    if (result_ready_) {
      const double c = (result_code_ == rclcpp_action::ResultCode::SUCCEEDED)
        ? pathLength(result_path_)
        : euclid(pairs_[pair_idx_]);
      costs_[pairs_[pair_idx_]] = c;
      advance();
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    if (accepted_ && gh_) {client_->async_cancel_goal(gh_);}
  }

private:
  void advance()
  {
    inflight_ = false;
    accepted_ = false;
    result_ready_ = false;
    gh_.reset();
    ++pair_idx_;
  }

  void sendQuery(const std::pair<int, int> & p)
  {
    ComputePath::Goal goal;
    goal.use_start = true;
    goal.start = (p.first == -1) ? start_ : goals_[p.first];
    goal.goal = goals_[p.second];
    goal.planner_id = planner_id_;

    accepted_ = false;
    result_ready_ = false;
    rclcpp_action::Client<ComputePath>::SendGoalOptions opts;
    opts.result_callback =
      [this](const GoalHandle::WrappedResult & r) {
        result_ready_ = true;
        result_code_ = r.code;
        if (r.result) {result_path_ = r.result->path;}
      };
    future_gh_ = client_->async_send_goal(goal, opts);
    inflight_ = true;
  }

  static double pathLength(const nav_msgs::msg::Path & path)
  {
    double len = 0.0;
    for (size_t i = 1; i < path.poses.size(); ++i) {
      const auto & a = path.poses[i - 1].pose.position;
      const auto & b = path.poses[i].pose.position;
      len += std::hypot(b.x - a.x, b.y - a.y);
    }
    return len;
  }

  double euclid(const std::pair<int, int> & p) const
  {
    const Pose & a = (p.first == -1) ? start_ : goals_[p.first];
    const Pose & b = goals_[p.second];
    return poseDistance(a, b);
  }

  double orderCost(const std::vector<int> & order) const
  {
    double total = costs_.at({-1, order.front()});
    for (size_t k = 1; k < order.size(); ++k) {
      total += costs_.at({order[k - 1], order[k]});
    }
    return total;
  }

  void finish()
  {
    const int n = static_cast<int>(goals_.size());
    std::vector<int> idx(n);
    for (int i = 0; i < n; ++i) {idx[i] = i;}

    const double naive_c = orderCost(idx);   // as-received order

    std::vector<int> best = idx;
    double best_c = std::numeric_limits<double>::max();
    std::sort(idx.begin(), idx.end());
    do {
      const double c = orderCost(idx);
      if (c < best_c) {best_c = c; best = idx;}
    } while (std::next_permutation(idx.begin(), idx.end()));

    std::vector<Pose> ordered;
    ordered.reserve(n);
    for (int i : best) {ordered.push_back(goals_[i]);}

    setOutput("ordered_goals", ordered);
    setOutput("ordered_indices", best);
    setOutput("naive_cost", naive_c);
    setOutput("best_cost", best_c);
    setOutput("time_saving", best_c <= naive_c * (1.0 - saving_margin_));

    RCLCPP_INFO(
      node_->get_logger(),
      "[InterleavedPlanner] naive=%.2fm best=%.2fm saving=%.1f%% (planner=%s)",
      naive_c, best_c,
      naive_c > 0.0 ? 100.0 * (naive_c - best_c) / naive_c : 0.0,
      use_planner_ ? "nav2" : "euclidean");
  }

  rclcpp::Node::SharedPtr node_;
  RobotPose robot_pose_;
  rclcpp_action::Client<ComputePath>::SharedPtr client_;

  std::vector<Pose> goals_;
  Pose start_;
  bool use_planner_{true};
  std::string planner_id_;
  double saving_margin_{0.02};

  std::vector<std::pair<int, int>> pairs_;
  size_t pair_idx_{0};
  std::map<std::pair<int, int>, double> costs_;

  bool inflight_{false};
  bool accepted_{false};
  bool result_ready_{false};
  std::shared_future<GoalHandle::SharedPtr> future_gh_;
  GoalHandle::SharedPtr gh_;
  rclcpp_action::ResultCode result_code_{rclcpp_action::ResultCode::UNKNOWN};
  nav_msgs::msg::Path result_path_;
};

}  // namespace nav_bt_stack

#endif  // NAV_BT_STACK__INTERLEAVED_PLANNER_HPP_