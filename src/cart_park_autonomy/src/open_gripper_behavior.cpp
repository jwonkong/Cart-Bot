#include <string>
#include <memory>
#include <cmath>

#include "open_gripper_behavior.hpp"

OpenGripper::OpenGripper(
  const std::string &name,
  const BT::NodeConfiguration &config,
  rclcpp::Node::SharedPtr node_ptr)
  : BT::StatefulActionNode(name, config), node_ptr_(node_ptr), done_flag_(false)
{
  action_client_ptr_ = rclcpp_action::create_client<Open>(node_ptr_, "open");
}

BT::NodeStatus OpenGripper::onStart()
{
  // action client의 옵션 설정
  auto send_goal_options = rclcpp_action::Client<Open>::SendGoalOptions();
  send_goal_options.result_callback =
    std::bind(&OpenGripper::result_callback, this, std::placeholders::_1);
  send_goal_options.feedback_callback =
    std::bind(&OpenGripper::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.goal_response_callback =
    std::bind(&OpenGripper::goal_response_callback, this, std::placeholders::_1);

  // 메시지 저장
  auto goal_msg = Open::Goal();
  goal_msg.open = 1; // 1이면 열기

  // 메시지 보내기
  action_client_ptr_->async_send_goal(goal_msg, send_goal_options);
  RCLCPP_INFO(node_ptr_->get_logger(), "Sent Goal to Open!!\n");

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus OpenGripper::onRunning()
{
  if (done_flag_)
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "[%s] Goal reached\n", this->name().c_str());
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::RUNNING;
  }
}
void OpenGripper::goal_response_callback(const GoalHandleOpen::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Action Goal was rejected by server");
  } else {
    RCLCPP_INFO(node_ptr_->get_logger(), "Action Goal accepted by server, waiting for result");
  }
}
void OpenGripper::feedback_callback(GoalHandleOpen::SharedPtr, const std::shared_ptr<const Open::Feedback> feedback)
{
  RCLCPP_INFO(node_ptr_->get_logger(), "Action feedback: ");
}
void OpenGripper::result_callback(const GoalHandleOpen::WrappedResult &result)
{
  // If there is a result, we consider navigation completed.
  // bt_navigator only sends an empty message without status. Idk why though.
  RCLCPP_INFO(node_ptr_->get_logger(), "Action succeeded!!");
  if (result.result)
  {
    done_flag_ = true;
  }
}
