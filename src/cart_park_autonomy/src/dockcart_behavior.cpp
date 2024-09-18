#include "dockcart_behavior.hpp"
#include "yaml-cpp/yaml.h"
#include <string>
#include <future>


using namespace std::chrono_literals;

double weightedMovingAverage(const std::deque<double>& values) {
    const double alpha = 0.5;

    double weighted_sum = 0.0;
    double weight_sum = 0.0;
    int n = values.size();
    for (int i = 0; i < n; ++i) {
        double weight = std::pow(alpha, n - i);
        weighted_sum += weight * values[i];
        weight_sum += weight;
    }

    return weighted_sum / weight_sum;
}

DockCart::DockCart(const std::string &name,
                   const BT::NodeConfiguration &config,
                   rclcpp::Node::SharedPtr node_ptr)
    : BT::StatefulActionNode(name, config), node_ptr_(node_ptr)
{
  publisher_ = node_ptr_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  prev_cmd_angular_z.resize(10, 0.0);
  prev_angle_degrees.resize(10, 0.0);
}


BT::PortsList DockCart::providedPorts()
{
  BT::PortsList list;
  list.insert(BT::InputPort<int>("dock_in"));
  list.insert(BT::OutputPort<int>("dock_out"));
  return list;
}

BT::NodeStatus DockCart::onStart()
{
  DODOCK = 1;
//   subscription_ = node_ptr_->create_subscription<geometry_msgs::msg::PoseArray>(
//   "/aruco_poses",10, std::bind(&DockCart::pose_callback, this, std::placeholders::_1)
//    );
  subscription__ = node_ptr_->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>( ///
  "/aruco_markers",10, std::bind(&DockCart::pose_callback, this, std::placeholders::_1) ///
  );

  timer_ = node_ptr_->create_wall_timer(std::chrono::seconds(1), std::bind(&DockCart::timer_callback, this));

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DockCart::onRunning()
{
  if (SUCCESS)
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "[%s] DOCK SUCCESS\n", this->name());
    DODOCK = 0;
    SUCCESS = 0;
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::RUNNING;
  }
}

void DockCart::timer_callback()
{
    rclcpp::Time current_time = node_ptr_->now();

    if(current_time.seconds() - receive_time.seconds() >= 3) {
        geometry_msgs::msg::Twist cmd_vel_msg;
        cmd_vel_msg.linear.x = 0.0;
        cmd_vel_msg.linear.y = 0.0;
        cmd_vel_msg.linear.z = 0.0;
        cmd_vel_msg.angular.x = 0.0;
        cmd_vel_msg.angular.y = 0.0;
        cmd_vel_msg.angular.z = -(prev_cmd_angular_z[-1]/abs(prev_cmd_angular_z[-1])) * 0.3;
        publisher_->publish(cmd_vel_msg);
        prev_error = 0;
        integral = 0;
    }
}   

void DockCart::pose_callback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg)
{
    receive_time = node_ptr_->now();

    if (DODOCK == 1)
    {
        int id = 0;

        for (size_t i = 0; i < msg->marker_ids.size(); ++i)
        {
            if(msg->marker_ids[i] == 0) {
                id = i;
                break;
            }
        }

        if (msg->poses.size() > 0) {
            // Data set
            double pose_x = msg->poses[id].position.x;
            double pose_z = msg->poses[id].position.z;
            double pose_qx = msg->poses[id].orientation.x;
            double pose_qy = msg->poses[id].orientation.y;
            double pose_qz = msg->poses[id].orientation.z;
            double pose_qw = msg->poses[id].orientation.w;

            // Quat to Euler
            double object_orientation[4] = {pose_qx, pose_qy, pose_qz, pose_qw};
            double rotation_matrix[3][3];
            rotation_matrix[0][0] = 1 - 2 * (object_orientation[2] * object_orientation[2] + object_orientation[3] * object_orientation[3]);
            rotation_matrix[0][1] = 2 * (object_orientation[1] * object_orientation[2] - object_orientation[3] * object_orientation[0]);
            rotation_matrix[0][2] = 2 * (object_orientation[1] * object_orientation[3] + object_orientation[2] * object_orientation[0]);
            rotation_matrix[1][0] = 2 * (object_orientation[1] * object_orientation[2] + object_orientation[3] * object_orientation[0]);
            rotation_matrix[1][1] = 1 - 2 * (object_orientation[1] * object_orientation[1] + object_orientation[3] * object_orientation[3]);
            rotation_matrix[1][2] = 2 * (object_orientation[2] * object_orientation[3] - object_orientation[1] * object_orientation[0]);
            rotation_matrix[2][0] = 2 * (object_orientation[1] * object_orientation[3] - object_orientation[2] * object_orientation[0]);
            rotation_matrix[2][1] = 2 * (object_orientation[2] * object_orientation[3] + object_orientation[1] * object_orientation[0]);
            rotation_matrix[2][2] = 1 - 2 * (object_orientation[1] * object_orientation[1] + object_orientation[2] * object_orientation[2]);

            // Extract direction angle
            double object_direction[3] = {rotation_matrix[0][0], rotation_matrix[1][0], rotation_matrix[2][0]};
            double my_direction[3] = {1, 0, 0};
            double dot_product = object_direction[0] * my_direction[0] + object_direction[1] * my_direction[1] + object_direction[2] * my_direction[2];
            double object_direction_length = sqrt(object_direction[0] * object_direction[0] + object_direction[1] * object_direction[1] + object_direction[2] * object_direction[2]);
            double my_direction_length = sqrt(my_direction[0] * my_direction[0] + my_direction[1] * my_direction[1] + my_direction[2] * my_direction[2]);
            double angle_radians = acos(dot_product / (object_direction_length * my_direction_length));
            double angle_degree = angle_radians * 180 / M_PI;

            // Filtering angle degree
            prev_angle_degrees.push_back(angle_degree);
            double filtered_angle_degree = weightedMovingAverage(prev_angle_degrees);

            // PID control
            double error = -0.01 - pose_x;
            integral += error;
            double derivative = error - prev_error;

            double angular_z = kp * error + ki * integral + kd * derivative;

            geometry_msgs::msg::Twist cmd_vel_msg;
            cmd_vel_msg.linear.x = -0.2;
            cmd_vel_msg.linear.y = 0.0;
            cmd_vel_msg.linear.z = 0.0;
            cmd_vel_msg.angular.x = 0.0;
            cmd_vel_msg.angular.y = 0.0;
            cmd_vel_msg.angular.z = angular_z;

            // Try docking
            if (pose_z < 0.65 && !SUCCESS && !AGAIN) {
                // Docking success
                if (pose_z < 0.65 && abs(error) < 0.04 && filtered_angle_degree < 16.0) {
                    cnt_near++;
                    if (filtered_angle_degree < 16.0 && cnt_near >= 3) {
                        cmd_vel_msg.linear.x = 0.0;
                        cmd_vel_msg.angular.z = 0.0;
                        publisher_->publish(cmd_vel_msg);
                        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                        cmd_vel_msg.linear.x = -0.4;
                        cmd_vel_msg.angular.z = 0.0;
                        while (cnt_time < 100000) {
                            cnt_time++;
                            publisher_->publish(cmd_vel_msg);
                        }
                        cmd_vel_msg.linear.x = 0.0;
                        publisher_->publish(cmd_vel_msg);
                        SUCCESS = true;
                    }
                // Set direction
                } else if (pose_z < 0.65 && (error >= 0.04 && filtered_angle_degree >= 16.0) && ccnt < 60) {
                    cmd_vel_msg.linear.x = 0.0;
                    cmd_vel_msg.angular.z = angular_z;
                    ccnt++;

                // Retry
                } else {
                    ccnt = 0;
                    prev_error = 0.0;
                    integral = 0.0;
                    AGAIN = true;
                }
            }

            // Back position to dock again
            if (AGAIN) {
                RCLCPP_INFO(node_ptr_->get_logger(), "AGAIN");


                if (pose_z < 0.65) {
                    cmd_vel_msg.linear.x = 0.2;
                    // test = filtered_angle_degree;
                }

                else if ((pose_z <= 1.2 && error < 0.04 && filtered_angle_degree < 16.0) || pose_z > 1.2) {
                    AGAIN = false;
                    REV = true;
                    prev_error = 0.0;
                    integral = 0.0;
                    cnt_near = 0;
                    cmd_vel_msg.linear.x = 0.0;
                    cmd_vel_msg.angular.z = 0.0;
                }

                else if (filtered_angle_degree <= 27.0 && REV) {
                    cmd_vel_msg.linear.x = 0.2;
                    cmd_vel_msg.angular.z = -angular_z;
                    if (abs(error) >= 0.23) {
                        REV = false;
                    }
                }

                else {
                    cmd_vel_msg.linear.x = 0.2;
                    cmd_vel_msg.angular.z = angular_z;
                }
            }

            // // Docking success
            // if (SUCCESS && pose_z < 0.65) {
            //     std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            //     cmd_vel_msg.linear.x = 1.0;
            //     cmd_vel_msg.angular.z = 0.0;
            //     while (cnt_num < 100000) {
            //         cnt_num++;
            //         publisher_->publish(cmd_vel_msg);
            //     }

            //     // rclcpp::shutdown();
            // }

            // Set max, min cmd
            if (cmd_vel_msg.angular.z > max_angular_z) {
                cmd_vel_msg.angular.z = max_angular_z;
            } else if (cmd_vel_msg.angular.z < -max_angular_z) {
                cmd_vel_msg.angular.z = -max_angular_z;
            }

            if (std::abs(cmd_vel_msg.angular.z) < min_angular_z) {
                if (cmd_vel_msg.angular.z > 0) {
                    cmd_vel_msg.angular.z = min_angular_z;
                } else if (cmd_vel_msg.angular.z < 0) {
                    cmd_vel_msg.angular.z = -min_angular_z;
                }
            }

            // pub
            publisher_->publish(cmd_vel_msg);
            RCLCPP_INFO(node_ptr_->get_logger(), "angle: %f", filtered_angle_degree);
            RCLCPP_INFO(node_ptr_->get_logger(), "Distance: %f", pose_z);
            RCLCPP_INFO(node_ptr_->get_logger(), "Error: %f", error);
            RCLCPP_INFO(node_ptr_->get_logger(), "Cmd angular z: %f", cmd_vel_msg.angular.z);
            RCLCPP_INFO(node_ptr_->get_logger(), " ");

            // Save previous value
            prev_error = error;
            prev_cmd_angular_z.push_back(cmd_vel_msg.angular.z);
            prev_filtered_angle_degree = filtered_angle_degree;
        }

    }
}