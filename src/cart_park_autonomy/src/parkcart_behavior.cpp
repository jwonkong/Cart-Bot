#include "parkcart_behavior.hpp"
#include "yaml-cpp/yaml.h"
#include <string>
#include <future>

ParkCart::ParkCart(const std::string &name,
                   const BT::NodeConfiguration &config,
                   rclcpp::Node::SharedPtr node_ptr)
    : BT::StatefulActionNode(name, config), node_ptr_(node_ptr)
{
  action_client_ptr_ = rclcpp_action::create_client<NavigateToPose>(node_ptr_, "/navigate_to_pose");
  done_flag_ = false;
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_ptr_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}


BT::PortsList ParkCart::providedPorts()
{
  BT::PortsList list;
  list.insert(BT::InputPort<geometry_msgs::msg::TransformStamped>("part_loc_in"));
  list.insert(BT::OutputPort<int>("park_loc_out"));
  return list;
}

BT::NodeStatus ParkCart::onStart()
{
  //auto cart_loc = getInput<geometry_msgs::msg::TransformStamped>("cart_loc_in");
  //geometry_msgs::msg::TransformStamped t = cart_loc.value();
  // setup action client
  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_goal_options.result_callback = std::bind(&ParkCart::nav_to_pose_callback, this, std::placeholders::_1);

  //***************************주차지점을 직접 지정!!!!!!!**********************************
  auto goal_msg = NavigateToPose::Goal();
  goal_msg.pose.header.frame_id = "map";
  goal_msg.pose.pose.position.x = 1;
  goal_msg.pose.pose.position.y = 0;

  tf2::Quaternion q(0,0,0,1);
  q.normalize();
  goal_msg.pose.pose.orientation = tf2::toMsg(q);

  // send pose
  done_flag_ = false;
  action_client_ptr_->async_send_goal(goal_msg, send_goal_options);
  RCLCPP_INFO(node_ptr_->get_logger(), "Sent Goal to Nav2\n");
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ParkCart::onRunning()
{
  if (done_flag_)
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "[%s] PARK SUCCESS\n", this->name());
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::RUNNING;
  }
}

void ParkCart::nav_to_pose_callback(const GoalHandleNav::WrappedResult &result)
{
  // If there is a result, we consider navigation completed.
  // bt_navigator only sends an empty message without status. Idk why though.

  if (result.result)
  {
    done_flag_ = true;
  }
}
