#include <string>
#include <atomic>
#include <memory>

#include "my_action_interface/action/open.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "behaviortree_cpp_v3/action_node.h"

class OpenGripper : public BT::StatefulActionNode
{
public:
  OpenGripper(
    const std::string &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node::SharedPtr node_ptr);

  using Open = my_action_interface::action::Open;
  using GoalHandleOpen = rclcpp_action::ClientGoalHandle<Open>;

  rclcpp::Node::SharedPtr node_ptr_;
  rclcpp_action::Client<Open>::SharedPtr action_client_ptr_;
  bool done_flag_;

  // Method overrides
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override{};

  // Action Client callback
  void goal_response_callback(const GoalHandleOpen::SharedPtr & goal_handle);
  void feedback_callback(GoalHandleOpen::SharedPtr, const std::shared_ptr<const Open::Feedback> feedback);
  void result_callback(const GoalHandleOpen::WrappedResult &result);
};

