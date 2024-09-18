#include "spin_back_behavior.hpp"
#include "yaml-cpp/yaml.h"
#include <string>
#include <future>

SpinBack::SpinBack(
  const std::string &name,
  const BT::NodeConfiguration &config,
  rclcpp::Node::SharedPtr node_ptr)
  : BT::StatefulActionNode(name, config), node_ptr_(node_ptr), done_flag_(false)
{
  action_client_ptr_ = rclcpp_action::create_client<BackUp>(node_ptr_, "open");
}

BT::NodeStatus SpinBack::onStart()
{
  // action client의 옵션 설정
  auto send_goal_options = rclcpp_action::Client<BackUp>::SendGoalOptions();
  send_goal_options.result_callback =
    std::bind(&SpinBack::result_callback, this, std::placeholders::_1);
  send_goal_options.feedback_callback =
    std::bind(&SpinBack::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.goal_response_callback =
    std::bind(&SpinBack::goal_response_callback, this, std::placeholders::_1);

  // 메시지 저장
  auto goal_msg = BackUp::Goal();
  goal_msg.target.x = 2; // 2m 뒤로 후진
  goal_msg.target.y = 0.0;
  goal_msg.target.z = 0.0;
  goal_msg.speed = 1; // 1m/s
  goal_msg.time_allowance = rclcpp::Duration::from_seconds(10); // 3초

  // 메시지 보내기
  action_client_ptr_->async_send_goal(goal_msg, send_goal_options);
  RCLCPP_INFO(node_ptr_->get_logger(), "Sent Goal to Open!!\n");

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SpinBack::onRunning()
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
void SpinBack::goal_response_callback(const GoalHandleSpinBack::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Action Goal was rejected by server");
  } else {
    RCLCPP_INFO(node_ptr_->get_logger(), "Action Goal accepted by server, waiting for result");
  }
}
void SpinBack::feedback_callback(GoalHandleSpinBack::SharedPtr, const std::shared_ptr<const BackUp::Feedback> feedback)
{
  RCLCPP_INFO(node_ptr_->get_logger(), "Action feedback: ");
}
void SpinBack::result_callback(const GoalHandleSpinBack::WrappedResult &result)
{
  // If there is a result, we consider navigation completed.
  // bt_navigator only sends an empty message without status. Idk why though.
  RCLCPP_INFO(node_ptr_->get_logger(), "Action succeeded!!");
  if (result.result)
  {
    done_flag_ = true;
  }
}
