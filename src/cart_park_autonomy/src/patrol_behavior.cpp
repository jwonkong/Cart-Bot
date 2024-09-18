#include "patrol_behavior.hpp"
#include "yaml-cpp/yaml.h"
#include <string>
#include <future>

Patrol::Patrol(const std::string &name,
                   const BT::NodeConfiguration &config,
                   rclcpp::Node::SharedPtr node_ptr)
    : BT::StatefulActionNode(name, config), node_ptr_(node_ptr)
{
  action_client_ptr_ = rclcpp_action::create_client<NavigateToPose>(node_ptr_, "/navigate_to_pose");
  done_flag_ = false;
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_ptr_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

BT::PortsList Patrol::providedPorts()
{
  BT::PortsList list;
  list.insert(BT::InputPort<int>("loc_in"));
  list.insert(BT::OutputPort<int>("loc_out"));
  list.insert(BT::OutputPort<geometry_msgs::msg::TransformStamped>("cart_loc_out"));
  return list;
}

BT::NodeStatus Patrol::onStart()
{
  // Get location key from port and read YAML file
  auto loc = getInput<int>("loc_in");
  std::string index = location + std::to_string(loc.value());
  count = loc.value();
  if(count<20)
  {
    count++;
  } else{
    count = 1;
  }
  setOutput("loc_out", count);

  const std::string location_file = node_ptr_->get_parameter("location_file").as_string();
  YAML::Node locations = YAML::LoadFile(location_file);
  std::vector<float> pose = locations[index].as<std::vector<float>>();

  // setup action client
  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_goal_options.result_callback = std::bind(&Patrol::nav_to_pose_callback, this, std::placeholders::_1);

  // make pose
  auto goal_msg = NavigateToPose::Goal();
  goal_msg.pose.header.frame_id = "map";
  goal_msg.pose.pose.position.x = pose[0];
  goal_msg.pose.pose.position.y = pose[1];

  tf2::Quaternion q;
  q.setRPY(0, 0, pose[2]);
  q.normalize(); // todo: why?
  goal_msg.pose.pose.orientation = tf2::toMsg(q);

  // send pose
  done_flag_ = false;
  action_client_ptr_->async_send_goal(goal_msg, send_goal_options);
  RCLCPP_INFO(node_ptr_->get_logger(), "PATROL : Sent Goal to Nav2\n");
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Patrol::onRunning()
{
  geometry_msgs::msg::TransformStamped t0;
  geometry_msgs::msg::TransformStamped t1;
  geometry_msgs::msg::TransformStamped t2;
  geometry_msgs::msg::TransformStamped t3;

  try {
        t0 = tf_buffer_->lookupTransform(
          "map", "marker_0_map",
          tf2::TimePointZero);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_INFO(
                   node_ptr_->get_logger(), "PATROL : Could not transform %s to %s: %s",
            "map", "marker_0", ex.what());
      }

  if(t0.header.stamp.sec != 0)
  {
    setOutput("cart_loc_out", t0);
    RCLCPP_INFO(node_ptr_->get_logger(), "PATROL : Patroll OUT\n");
    RCLCPP_INFO(
    node_ptr_->get_logger(),
    "TransformStamped: child_frame_id=%s, frame_id=%s, timestamp=%d.%d, x=%f, y=%f, z=%f, qx=%f, qy=%f, qz=%f, qw=%f",
    t0.child_frame_id.c_str(),
    t0.header.frame_id.c_str(),
    t0.header.stamp.sec,
    t0.header.stamp.nanosec,
    t0.transform.translation.x,
    t0.transform.translation.y,
    t0.transform.translation.z,
    t0.transform.rotation.x,
    t0.transform.rotation.y,
    t0.transform.rotation.z,
    t0.transform.rotation.w);
    return BT::NodeStatus::SUCCESS;
  }
  
  try {
        t1 = tf_buffer_->lookupTransform(
          "map", "marker_1_map",
          tf2::TimePointZero);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_INFO(
                   node_ptr_->get_logger(), "PATROL : Could not transform %s to %s: %s",
            "map", "marker_1", ex.what());
      }
  if(t1.header.stamp.sec != 0)
  {
    setOutput("cart_loc_out", t1);
    RCLCPP_INFO(node_ptr_->get_logger(), "PATROL : Patroll OUT\n");
    RCLCPP_INFO(
    node_ptr_->get_logger(),
    "TransformStamped: child_frame_id=%s, frame_id=%s, timestamp=%d.%d, x=%f, y=%f, z=%f, qx=%f, qy=%f, qz=%f, qw=%f",
    t1.child_frame_id.c_str(),
    t1.header.frame_id.c_str(),
    t1.header.stamp.sec,
    t1.header.stamp.nanosec,
    t1.transform.translation.x,
    t1.transform.translation.y,
    t1.transform.translation.z,
    t1.transform.rotation.x,
    t1.transform.rotation.y,
    t1.transform.rotation.z,
    t1.transform.rotation.w);
    return BT::NodeStatus::SUCCESS;
  }
  
  try {
        t2 = tf_buffer_->lookupTransform(
          "map", "marker_2_map",
          tf2::TimePointZero);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_INFO(
                   node_ptr_->get_logger(), "PATROL : Could not transform %s to %s: %s",
            "map", "marker_2", ex.what());
      }
  if(t2.header.stamp.sec != 0)
  {
    setOutput("cart_loc_out", t2);
    RCLCPP_INFO(node_ptr_->get_logger(), "PATROL : Patroll OUT\n");
    RCLCPP_INFO(
    node_ptr_->get_logger(),
    "TransformStamped: child_frame_id=%s, frame_id=%s, timestamp=%d.%d, x=%f, y=%f, z=%f, qx=%f, qy=%f, qz=%f, qw=%f",
    t2.child_frame_id.c_str(),
    t2.header.frame_id.c_str(),
    t2.header.stamp.sec,
    t2.header.stamp.nanosec,
    t2.transform.translation.x,
    t2.transform.translation.y,
    t2.transform.translation.z,
    t2.transform.rotation.x,
    t2.transform.rotation.y,
    t2.transform.rotation.z,
    t2.transform.rotation.w);
    return BT::NodeStatus::SUCCESS;
  }
 
  try {
        t3 = tf_buffer_->lookupTransform(
          "map", "marker_3_map",
          tf2::TimePointZero);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_INFO(
                   node_ptr_->get_logger(), "PATROL : Could not transform %s to %s: %s",
            "map", "marker_3", ex.what());
      }

  if(t3.header.stamp.sec != 0)
  {
    setOutput("cart_loc_out", t3);
    RCLCPP_INFO(node_ptr_->get_logger(), "PATROL : Patroll OUT\n");
    RCLCPP_INFO(
    node_ptr_->get_logger(),
    "TransformStamped: child_frame_id=%s, frame_id=%s, timestamp=%d.%d, x=%f, y=%f, z=%f, qx=%f, qy=%f, qz=%f, qw=%f",
    t3.child_frame_id.c_str(),
    t3.header.frame_id.c_str(),
    t3.header.stamp.sec,
    t3.header.stamp.nanosec,
    t3.transform.translation.x,
    t3.transform.translation.y,
    t3.transform.translation.z,
    t3.transform.rotation.x,
    t3.transform.rotation.y,
    t3.transform.rotation.z,
    t3.transform.rotation.w);
    return BT::NodeStatus::SUCCESS;
  }

  if (done_flag_)
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "[%s] PATROL : Goal reached\n", this->name().c_str());
    return BT::NodeStatus::FAILURE;
    //return BT::NodeStatus::FAILURE;
  }
  else
  {
    return BT::NodeStatus::RUNNING;
  }
}

void Patrol::nav_to_pose_callback(const GoalHandleNav::WrappedResult &result)
{
  // If there is a result, we consider navigation completed.
  // bt_navigator only sends an empty message without status. Idk why though.

  if (result.result)
  {
    done_flag_ = true;
  }
}

