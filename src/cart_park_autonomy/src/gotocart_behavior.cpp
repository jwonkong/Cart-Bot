#include "gotocart_behavior.hpp"
#include "yaml-cpp/yaml.h"
#include <string>
#include <future>

void GoToCart::ReturnMeanGoal(NavigateToPose::Goal &goal)
{
  float trans_x = 0;
  float trans_y = 0;
  float trans_z = 0;
  float rot_x = 0;
  float rot_y = 0;
  float rot_z = 0;
  float rot_w = 0;

  for (int i = 0;i < map_cart_arr.size();i++)
  {
    trans_x += map_cart_arr[i].transform.translation.x;
    trans_y += map_cart_arr[i].transform.translation.y;
    trans_z += map_cart_arr[i].transform.translation.z;

    rot_x += map_cart_arr[i].transform.rotation.x;
    rot_y += map_cart_arr[i].transform.rotation.y;
    rot_z += map_cart_arr[i].transform.rotation.z;
    rot_w += map_cart_arr[i].transform.rotation.w;
  }

  goal.pose.pose.position.x = trans_x / map_cart_arr.size();
  goal.pose.pose.position.y = trans_y / map_cart_arr.size();
  goal.pose.pose.position.z = trans_z / map_cart_arr.size();

  tf2::Quaternion q(rot_x / map_cart_arr.size(),rot_y / map_cart_arr.size(),
              rot_z / map_cart_arr.size() ,rot_w / map_cart_arr.size());
  q.normalize();
  goal.pose.pose.orientation = tf2::toMsg(q);
  goal.pose.header.frame_id = "map";
  map_cart_arr.clear();
}

GoToCart::GoToCart(const std::string &name,
                   const BT::NodeConfiguration &config,
                   rclcpp::Node::SharedPtr node_ptr)
    : BT::StatefulActionNode(name, config), node_ptr_(node_ptr)
{
  action_client_ptr_ = rclcpp_action::create_client<NavigateToPose>(node_ptr_, "/navigate_to_pose");
  done_flag_ = false;
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_ptr_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}


BT::PortsList GoToCart::providedPorts()
{
  BT::PortsList list;
  list.insert(BT::InputPort<geometry_msgs::msg::TransformStamped>("cart_loc_in"));
  list.insert(BT::OutputPort<int>("loc_out"));
  return list;
}

BT::NodeStatus GoToCart::onStart()
{
  auto cart_loc = getInput<geometry_msgs::msg::TransformStamped>("cart_loc_in");
  geometry_msgs::msg::TransformStamped t = cart_loc.value();
  // setup action client
  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_goal_options.result_callback = std::bind(&GoToCart::nav_to_pose_callback, this, std::placeholders::_1);

  // make pose
  auto goal_msg = NavigateToPose::Goal();
  goal_msg.pose.header.frame_id = "map";
  goal_msg.pose.pose.position.x = t.transform.translation.x;
  goal_msg.pose.pose.position.y = t.transform.translation.y;

  tf2::Quaternion q(t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w);
  q.normalize();
  goal_msg.pose.pose.orientation = tf2::toMsg(q);

  // send pose
  done_flag_ = false;
  action_client_ptr_->async_send_goal(goal_msg, send_goal_options);
  RCLCPP_INFO(node_ptr_->get_logger(), "Sent Goal to Nav2\n");
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoToCart::onRunning()
{
  geometry_msgs::msg::TransformStamped map_cart;
  geometry_msgs::msg::TransformStamped robot_cart;

  if (Regoal == false)
  {
    try {
        map_cart = tf_buffer_->lookupTransform(
          "laser_frame", "marker_0_map",
          tf2::TimePointZero);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_INFO(
                   node_ptr_->get_logger(), "Could not transform %s to %s: %s",
            "map", "marker_0_map", ex.what());
      }
    if(map_cart.header.stamp.sec != 0)
    {
      try {
          robot_cart = tf_buffer_->lookupTransform(
            "map", "marker_0_map",
            tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
          RCLCPP_INFO(
                    node_ptr_->get_logger(), "Could not transform %s to %s: %s",
              "map", "marker_0_map", ex.what());
        }
      map_cart_arr.push_back(map_cart);

      if(robot_cart.header.stamp.sec != 0)
      {
        if(sqrt(robot_cart.transform.translation.x*robot_cart.transform.translation.x
            +robot_cart.transform.translation.y*robot_cart.transform.translation.y
            +robot_cart.transform.translation.z*robot_cart.transform.translation.z) < 4)
            {

              auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_goal_options.result_callback = std::bind(&GoToCart::nav_to_pose_callback, this, std::placeholders::_1);
              auto goal_msg = NavigateToPose::Goal();
              ReturnMeanGoal(goal_msg);

              // send pose
              action_client_ptr_->async_send_goal(goal_msg, send_goal_options);
              RCLCPP_INFO(node_ptr_->get_logger(), "RESENT Goal to Nav2\n");
              Regoal = true;
              return BT::NodeStatus::RUNNING;
        }

        try {
            robot_cart = tf_buffer_->lookupTransform(
              "map", "marker_1_map",
              tf2::TimePointZero);
          } catch (const tf2::TransformException & ex) {
            RCLCPP_INFO(
                      node_ptr_->get_logger(), "Could not transform %s to %s: %s",
                "map", "marker_1_map", ex.what());
          }
        if(robot_cart.header.stamp.sec != 0)
        {
          if(sqrt(robot_cart.transform.translation.x*robot_cart.transform.translation.x
              +robot_cart.transform.translation.y*robot_cart.transform.translation.y
              +robot_cart.transform.translation.z*robot_cart.transform.translation.z) < 4)
              {
                auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(&GoToCart::nav_to_pose_callback, this, std::placeholders::_1);
                auto goal_msg = NavigateToPose::Goal();
                ReturnMeanGoal(goal_msg);

                // send pose
                action_client_ptr_->async_send_goal(goal_msg, send_goal_options);
                RCLCPP_INFO(node_ptr_->get_logger(), "RESENT Goal to Nav2\n");
                Regoal = true;
                return BT::NodeStatus::RUNNING;
              }
          }

          try {
              robot_cart = tf_buffer_->lookupTransform(
                "map", "marker_2_map",
                tf2::TimePointZero);
            } catch (const tf2::TransformException & ex) {
              RCLCPP_INFO(
                        node_ptr_->get_logger(), "Could not transform %s to %s: %s",
                  "map", "marker_2_map", ex.what());
            }
          if(robot_cart.header.stamp.sec != 0)
          {
            if(sqrt(robot_cart.transform.translation.x*robot_cart.transform.translation.x
                +robot_cart.transform.translation.y*robot_cart.transform.translation.y
                +robot_cart.transform.translation.z*robot_cart.transform.translation.z) < 4)
                {
                  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
      send_goal_options.result_callback = std::bind(&GoToCart::nav_to_pose_callback, this, std::placeholders::_1);
                  auto goal_msg = NavigateToPose::Goal();
                  ReturnMeanGoal(goal_msg);

                  // send pose
                  action_client_ptr_->async_send_goal(goal_msg, send_goal_options);
                  RCLCPP_INFO(node_ptr_->get_logger(), "RESENT Goal to Nav2\n");
                  Regoal = true;
                  return BT::NodeStatus::RUNNING;
                }
            }

          try {
              robot_cart = tf_buffer_->lookupTransform(
                "map", "marker_3_map",
                tf2::TimePointZero);
            } catch (const tf2::TransformException & ex) {
              RCLCPP_INFO(
                        node_ptr_->get_logger(), "Could not transform %s to %s: %s",
                  "map", "marker_3_map", ex.what());
            }
          if(robot_cart.header.stamp.sec != 0)
          {
            if(sqrt(robot_cart.transform.translation.x*robot_cart.transform.translation.x
                +robot_cart.transform.translation.y*robot_cart.transform.translation.y
                +robot_cart.transform.translation.z*robot_cart.transform.translation.z) < 4)
                {
                  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
      send_goal_options.result_callback = std::bind(&GoToCart::nav_to_pose_callback, this, std::placeholders::_1);
                  auto goal_msg = NavigateToPose::Goal();
                  ReturnMeanGoal(goal_msg);

                  // send pose
                  action_client_ptr_->async_send_goal(goal_msg, send_goal_options);
                  RCLCPP_INFO(node_ptr_->get_logger(), "RESENT Goal to Nav2\n");
                  Regoal = true;
                  return BT::NodeStatus::RUNNING;
                }
            }
      }
    }
  }

  if (done_flag_)
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "[%s] Goal reached\n", this->name());
    //Regoal = false;
    return BT::NodeStatus::SUCCESS;
    //return BT::NodeStatus::FAILURE;
  }
  else
  {
    return BT::NodeStatus::RUNNING;
  }
}

void GoToCart::nav_to_pose_callback(const GoalHandleNav::WrappedResult &result)
{
  // If there is a result, we consider navigation completed.
  // bt_navigator only sends an empty message without status. Idk why though.

  if (result.result)
  {
    done_flag_ = true;
  }
}
