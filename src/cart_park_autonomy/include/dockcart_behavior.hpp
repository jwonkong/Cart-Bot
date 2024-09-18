#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>
#include <deque>
#include <vector>
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
using namespace std::chrono_literals;


class DockCart : public BT::StatefulActionNode
{
public:
  DockCart(const std::string &name,
           const BT::NodeConfiguration &config,
           rclcpp::Node::SharedPtr node_ptr);
  rclcpp::Node::SharedPtr node_ptr_;
  // Method overrides
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override{};
  static BT::PortsList providedPorts();

private:
  // rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_;
  rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr subscription__;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  void pose_callback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg);
  void timer_callback();

  // Timer
  rclcpp::Time receive_time;

  // PID param
  double prev_error;
  double integral;
  double kp = 5;
  double ki = 0.6;
  double kd = 120;

  // Max, Min cmd
  double max_angular_z = 3.0;
  double min_angular_z = 0.1;

  // Counter
  int cnt_num;
  int cnt_near;
  int cnt_time;

  bool DODOCK;
  bool SUCCESS;
  bool AGAIN;
  bool REV = true;
  int cnt1;
  int cnt2;
  int ccnt;
  double test;
  std::deque<double> prev_cmd_angular_z;
  std::deque<double> prev_angle_degrees;
  double prev_filtered_angle_degree;
};

