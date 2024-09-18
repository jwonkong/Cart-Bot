/*
 *  YDLIDAR SYSTEM
 *  YDLIDAR ROS 2 Node
 *
 *  Copyright 2017 - 2020 EAI TEAM
 *  http://www.eaibot.com
 *
 */

#ifdef _MSC_VER
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#endif

#include "src/CYdLidar.h"
#include <math.h>
#include <chrono>
#include <iostream>
#include <memory>

#include "laser_geometry/laser_geometry.hpp" //추가됨.
//#include "/home/younnj/hht_ws/src/laser_geometry-humble/src/laser_geometry.cpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>

#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/empty.hpp"
#include <vector>
#include <iostream>
#include <string>
#include <signal.h>
#include "rclcpp/duration.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>

#define ROS2Verision "1.0.1"

// sensor_msgs::msg::PointCloud2 laser2cloudmsg(sensor_msgs::msg::LaserScan laser)
// {
//   static laser_geometry::LaserProjection proeector;
//   sensor_msgs::msg::PointCloud2 pc2_dst;
//   proeector.projectLaser(laser, pc2_dst,-1,laser_geometry::channel_option::Intensity | laser_geometry::channel_option::Distance);
//   pc2_dst.header.frame_id = "map";

//   return pc2_dst;             //pointclode2 메시지가 전달되면 타임스탬프 쓰이는데 이에 관해 문제점 안 생기는지, frame_id에 대해 조사해보고 딱히 신경 안써줘도 되는지 여부 파악.
// }

// void quatReturner(std_msgs::msg::String tiltinfo)
// {
//   float tiltangle = std::stof(tiltinfo.data);

//   quat_start.setRPY(0,(tiltangle-90.0)/3.14159,0); //실제 몸체 제작 시 앞 뒤 방향을 반영하고 싶으면 이 부분의 부호를 바꿔주면 된다.

//   if(tiltangle > formerval)
//   {
//    quat_end.setRPY(0,(tiltangle+ANGLEINCRE-90.0)/3.14159,0);
//   }
//   else
//   {
//     quat_end.setRPY(0,(tiltangle+ANGLEINCRE-90.0)/3.14159,0);
//   }

//   formerval = tiltangle;
// }

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("ydlidar_ros2_driver_node");

  RCLCPP_INFO(node->get_logger(), "[YDLIDAR INFO] Current ROS Driver Version: %s\n", ((std::string)ROS2Verision).c_str());

  CYdLidar laser;
  std::string str_optvalue = "/dev/ttyUSB0";
  node->declare_parameter("port",rclcpp::ParameterValue(str_optvalue));
  node->get_parameter("port");
  ///lidar port
  laser.setlidaropt(LidarPropSerialPort, str_optvalue.c_str(), str_optvalue.size());

  ///ignore array
  str_optvalue = "";
  node->declare_parameter("ignore_array",rclcpp::ParameterValue(str_optvalue));
  node->get_parameter("ignore_array");
  laser.setlidaropt(LidarPropIgnoreArray, str_optvalue.c_str(), str_optvalue.size());

  std::string frame_id = "horizontal_laser_link";
  node->declare_parameter("frame_id",rclcpp::ParameterValue(frame_id));
  node->get_parameter("frame_id");

  //////////////////////int property/////////////////
  /// lidar baudrate
  int optval = 128000;
  node->declare_parameter("baudrate",rclcpp::ParameterValue(optval));
  node->get_parameter("baudrate");
  laser.setlidaropt(LidarPropSerialBaudrate, &optval, sizeof(int));
  /// tof lidar
  optval = 1;
  node->declare_parameter("lidar_type",rclcpp::ParameterValue(optval));
  node->get_parameter("lidar_type");
  laser.setlidaropt(LidarPropLidarType, &optval, sizeof(int));
  /// device type
  optval = 0;
  node->declare_parameter("device_type",rclcpp::ParameterValue(optval));
  node->get_parameter("device_type");
  laser.setlidaropt(LidarPropDeviceType, &optval, sizeof(int));
  /// sample rate
  optval = 5;
  node->declare_parameter("sample_rate",rclcpp::ParameterValue(optval));
  node->get_parameter("sample_rate");
  laser.setlidaropt(LidarPropSampleRate, &optval, sizeof(int));
  /// abnormal count
  optval = 4;
  node->declare_parameter("abnormal_check_count",rclcpp::ParameterValue(optval));
  node->get_parameter("abnormal_check_count");
  laser.setlidaropt(LidarPropAbnormalCheckCount, &optval, sizeof(int));

  /// Intenstiy bit count
  optval = 8;
  node->declare_parameter("intensity_bit",rclcpp::ParameterValue(optval));
  node->get_parameter("intensity_bit");
  laser.setlidaropt(LidarPropIntenstiyBit, &optval, sizeof(int));

  //////////////////////bool property/////////////////
  /// fixed angle resolution
  bool b_optvalue = true;
  node->declare_parameter("fixed_resolution",rclcpp::ParameterValue(b_optvalue));
  node->get_parameter("fixed_resolution");
  laser.setlidaropt(LidarPropFixedResolution, &b_optvalue, sizeof(bool));
  /// rotate 180
  b_optvalue = true;
  node->declare_parameter("reversion",rclcpp::ParameterValue(b_optvalue));
  node->get_parameter("reversion");
  laser.setlidaropt(LidarPropReversion, &b_optvalue, sizeof(bool));
  /// Counterclockwise
  b_optvalue = true;
  node->declare_parameter("inverted",rclcpp::ParameterValue(b_optvalue));
  node->get_parameter("inverted");
  laser.setlidaropt(LidarPropInverted, &b_optvalue, sizeof(bool));
  b_optvalue = true;
  node->declare_parameter("auto_reconnect",rclcpp::ParameterValue(b_optvalue));
  node->get_parameter("auto_reconnect");
  laser.setlidaropt(LidarPropAutoReconnect, &b_optvalue, sizeof(bool));
  /// one-way communication
  b_optvalue = true;
  node->declare_parameter("isSingleChannel",rclcpp::ParameterValue(b_optvalue));
  node->get_parameter("isSingleChannel");
  laser.setlidaropt(LidarPropSingleChannel, &b_optvalue, sizeof(bool));
  /// intensity
  b_optvalue = false;
  node->declare_parameter("intensity",rclcpp::ParameterValue(b_optvalue));
  node->get_parameter("intensity");
  laser.setlidaropt(LidarPropIntenstiy, &b_optvalue, sizeof(bool));
  /// Motor DTR
  b_optvalue = false;
  node->declare_parameter("support_motor_dtr",rclcpp::ParameterValue(b_optvalue));
  node->get_parameter("support_motor_dtr");
  laser.setlidaropt(LidarPropSupportMotorDtrCtrl, &b_optvalue, sizeof(bool));

  //////////////////////float property/////////////////
  /// unit: °
  float f_optvalue = 180.0f;
  node->declare_parameter("angle_max",rclcpp::ParameterValue(f_optvalue));
  node->get_parameter("angle_max");
  laser.setlidaropt(LidarPropMaxAngle, &f_optvalue, sizeof(float));
  f_optvalue = -180.0f;
  node->declare_parameter("angle_min",rclcpp::ParameterValue(f_optvalue));
  node->get_parameter("angle_min");
  laser.setlidaropt(LidarPropMinAngle, &f_optvalue, sizeof(float));
  /// unit: m
  f_optvalue = 12.0f;
  node->declare_parameter("range_max",rclcpp::ParameterValue(f_optvalue));
  node->get_parameter("range_max");
  laser.setlidaropt(LidarPropMaxRange, &f_optvalue, sizeof(float));
  f_optvalue = 0.1f;
  node->declare_parameter("range_min",rclcpp::ParameterValue(f_optvalue));
  node->get_parameter("range_min");
  laser.setlidaropt(LidarPropMinRange, &f_optvalue, sizeof(float));
  /// unit: Hz
  f_optvalue = 10.0f;
  node->declare_parameter("frequency",rclcpp::ParameterValue(f_optvalue));
  node->get_parameter("frequency");
  laser.setlidaropt(LidarPropScanFrequency, &f_optvalue, sizeof(float));

  bool invalid_range_is_inf = false;
  node->declare_parameter("invalid_range_is_inf",rclcpp::ParameterValue(invalid_range_is_inf));
  node->get_parameter("invalid_range_is_inf");


  bool ret = laser.initialize();
  if (ret) {
    ret = laser.turnOn();
  } else {
    RCLCPP_ERROR(node->get_logger(), "%s\n", laser.DescribeError());
  }

  //const auto QOS = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort().durability_volatile();
  auto laser_pub = node->create_publisher<sensor_msgs::msg::LaserScan>("/scan", rclcpp::SensorDataQoS());
                                                                                                         //laserscan 대신 pointcloud2를 퍼블리시 하는 것으로 바꿔야함
                                                                                                         //그리고 "scan" 대신 "/sync_scan_cloud_filtered"로 바꿔야함
  auto tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node);

  auto stop_scan_service =
    [&laser](const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::Empty::Request> req,
  std::shared_ptr<std_srvs::srv::Empty::Response> response) -> bool
  {
    return laser.turnOff();
  };

  auto stop_service = node->create_service<std_srvs::srv::Empty>("stop_scan",stop_scan_service);

  auto start_scan_service =
    [&laser](const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::Empty::Request> req,
  std::shared_ptr<std_srvs::srv::Empty::Response> response) -> bool
  {
    return laser.turnOn();
  };

  auto start_service = node->create_service<std_srvs::srv::Empty>("start_scan",start_scan_service);

  auto projector = std::make_shared<laser_geometry::LaserProjection>();

  rclcpp::WallRate loop_rate(20);

  while (ret && rclcpp::ok()) {

    LaserScan scan;

    if (laser.doProcessSimple(scan)) {

      auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();

      scan_msg->header.stamp.sec = RCL_NS_TO_S(scan.stamp);
      scan_msg->header.stamp.nanosec =  scan.stamp - RCL_S_TO_NS(scan_msg->header.stamp.sec);
      scan_msg->header.frame_id = "horizontal_laser_link";
      scan_msg->angle_min = scan.config.min_angle;
      scan_msg->angle_max = scan.config.max_angle;
      scan_msg->angle_increment = scan.config.angle_increment;
      scan_msg->scan_time = scan.config.scan_time;
      scan_msg->time_increment = scan.config.time_increment;
      scan_msg->range_min = scan.config.min_range;
      scan_msg->range_max = scan.config.max_range;

      int size = (scan.config.max_angle - scan.config.min_angle)/ scan.config.angle_increment + 1;
      scan_msg->ranges.resize(size);
      scan_msg->intensities.resize(size);
      for(size_t i=0; i < scan.points.size(); i++) {
        int index = std::ceil((scan.points[i].angle - scan.config.min_angle)/scan.config.angle_increment);
        if(index >=0 && index < size) {
          scan_msg->ranges[index] = scan.points[i].range;
          scan_msg->intensities[index] = scan.points[i].intensity;
        }
      }

      laser_pub->publish(*scan_msg);

    } else {
      RCLCPP_ERROR(node->get_logger(), "Failed to get scan");
    }
    if(!rclcpp::ok()) {
      break;
    }
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }


  RCLCPP_INFO(node->get_logger(), "[YDLIDAR INFO] Now YDLIDAR is stopping .......");
  laser.turnOff();
  laser.disconnecting();
  rclcpp::shutdown();

  return 0;
}

