
#include <iostream>  // Libreria standard per input/output (printf, std::cout, ecc.)
#include <rclcpp/rclcpp.hpp>
#include <livox_lidar_interfaces/srv/msg_enable_disable.hpp>
#include <livox_lidar_interfaces/msg/livox_info.hpp>
#include "livox_mid360_driver/livox_mid360_driver.hpp"
#include <livox_mid360_driver/livox_lidar_api.h>
#include <livox_mid360_driver/livox_lidar_def.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <arpa/inet.h>
#include <vector>

static constexpr int PUB_PERIOD_MS =5;
static constexpr int PUB_PERIOD_MS_IMU =1000;
static constexpr int PUB_PERIOD_MS_INFO =5000;


Mid360_Server::Mid360_Server()
: Node("server") // Costruzione del nodo ROS2 con nome "server"
{
  // Creazione del servizio /msg/Mid360_Server, legato al callback Mid360_Enable_Disable_clbk
  server_ = this->create_service<MsgEnableDisable>(
    "/msg/Mid360_Server",
    std::bind(
      &Mid360_Server::Mid360_Enable_Disable_clbk,  // Metodo da chiamare alle richieste
      this,
      std::placeholders::_1,
      std::placeholders::_2));                     // Placeholder per il parametro request

  RCLCPP_INFO(this->get_logger(), "Server initialized");  // Log di avvenuta inizializzazione

  // 3) Publisher PointCloud2
  pc2_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/msg_MID360/PointCloud2", rclcpp::QoS(10));
  RCLCPP_INFO(this->get_logger(), "Publisher PointCloud2 avviato");

  // 4) Publisher IMU
  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
    "/msg_MID360/IMU", rclcpp::QoS(10));
  RCLCPP_INFO(this->get_logger(), "Publisher IMU avviato");

    // 5) Publisher LivoxInfo
  info_pub_ = this->create_publisher<livox_lidar_interfaces::msg::LivoxInfo>(
    "/msg_MID360/INFO", rclcpp::QoS(10));
  RCLCPP_INFO(this->get_logger(), "Publisher LivoxInfo avviato");

  // 6) Timer PC2
  pub_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(PUB_PERIOD_MS),
    std::bind(&Mid360_Server::on_pub_timer, this));
  RCLCPP_INFO(this->get_logger(), "Timer di publish impostato a %d ms", PUB_PERIOD_MS);
  
  // 6) Timer IMU
  pub_timer_IMU_ = this->create_wall_timer(
    std::chrono::milliseconds(PUB_PERIOD_MS_IMU),
    std::bind(&Mid360_Server::on_pub_timer_IMU, this));
  RCLCPP_INFO(this->get_logger(), "Timer di publish IMU impostato a %d ms", PUB_PERIOD_MS_IMU);

  // 6) Timer INFO
  pub_timer_INFO_ = this->create_wall_timer(
    std::chrono::milliseconds(PUB_PERIOD_MS_INFO),
    std::bind(&Mid360_Server::on_pub_timer_INFO, this));
  RCLCPP_INFO(this->get_logger(), "Timer di publish INFOimpostato a %d ms", PUB_PERIOD_MS_INFO);
}
