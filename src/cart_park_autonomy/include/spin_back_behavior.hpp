#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/back_up.hpp"

#include <string>
#include "behaviortree_cpp_v3/action_node.h"

class SpinBack : public BT::StatefulActionNode
{
public:
  SpinBack(
    const std::string &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node::SharedPtr node_ptr);

  using BackUp = nav2_msgs::action::BackUp;
  using GoalHandleSpinBack = rclcpp_action::ClientGoalHandle<BackUp>;

  rclcpp::Node::SharedPtr node_ptr_;
  rclcpp_action::Client<BackUp>::SharedPtr action_client_ptr_;
  bool done_flag_;

  // Method overrides
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override{};

  // Action Client callback
  void goal_response_callback(const GoalHandleSpinBack::SharedPtr & goal_handle);
  void feedback_callback(GoalHandleSpinBack::SharedPtr, const std::shared_ptr<const BackUp::Feedback> feedback);
  void result_callback(const GoalHandleSpinBack::WrappedResult &result);
};
