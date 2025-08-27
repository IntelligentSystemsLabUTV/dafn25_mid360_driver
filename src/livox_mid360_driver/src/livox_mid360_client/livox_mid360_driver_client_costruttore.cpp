#include <iostream>
#include <stdexcept>

#include "livox_mid360_driver/livox_mid360_driver.hpp"
#include <livox_lidar_interfaces/srv/msg_enable_disable.hpp>
#include <livox_lidar_interfaces/msg/livox_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/string.hpp>
#include <arpa/inet.h>

Mid360_Client::Mid360_Client() : Node("client")
{
  // --- Client del servizio ---
  client_ = this->create_client<livox_lidar_interfaces::srv::MsgEnableDisable>("/msg/Mid360_Server");
  RCLCPP_INFO(this->get_logger(), "Client initialized");

  // --- Subscription PointCloud2 ---
  pc2_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/msg_MID360/PointCloud2", 10,
    std::bind(&Mid360_Client::pc2Callback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscription PointCloud2 avviata");

  // --- Subscription IMU ---
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/msg_MID360/IMU", 10,
    std::bind(&Mid360_Client::imuCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscription IMU avviata");

  // --- Subscription LivoxInfo ---
  info_sub_ = this->create_subscription<livox_lidar_interfaces::msg::LivoxInfo>(
    "/msg_MID360/INFO",
    rclcpp::QoS(10),
    std::bind(&Mid360_Client::infoCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscription LivoxInfo avviata");
}