#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <string>
#include "behaviortree_cpp_v3/action_node.h"

class Patrol : public BT::StatefulActionNode
{
public:
  Patrol(const std::string &name,
           const BT::NodeConfiguration &config,
           rclcpp::Node::SharedPtr node_ptr);

  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  rclcpp::Node::SharedPtr node_ptr_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_ptr_;
  bool done_flag_;

  // Method overrides
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override{};

  static BT::PortsList providedPorts();

  // Action Client callback
  void nav_to_pose_callback(const GoalHandleNav::WrappedResult &result);
private:
  int count;
  std::string location = "location";
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

class FirstLocSetter : public BT::SyncActionNode
{
public:
  FirstLocSetter(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  { }

  static BT::PortsList providedPorts()
  {
    return {BT::OutputPort<int>("loc")};
  }

  // This Action writes a value into the port "text"
  BT::NodeStatus tick() override
  {
    // the output may change at each tick(). Here we keep it simple.
    setOutput("loc", "location1");
    return BT::NodeStatus::SUCCESS;
  }
};

class Retry : public BT::RetryNode
{
  public:
    Retry(const std::string& name, const BT::NodeConfiguration& config)
      : BT::RetryNode(name, 30)
    {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<int>("RetryTimes")};
  }
};
