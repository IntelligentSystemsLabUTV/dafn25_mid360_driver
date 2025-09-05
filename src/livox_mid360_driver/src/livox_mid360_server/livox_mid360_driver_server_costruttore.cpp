
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



Mid360_Server::Mid360_Server()
: Node("server") // Costruzione del nodo ROS2 con nome "server"
{


  //MODIFICA 
  this->declare_parameter<bool>("autostart", false);
  this->declare_parameter<std::string>("livox_cfg_path", "");

  bool autostart;
  std::string livox_cfg_path;
  this->get_parameter("autostart", autostart);
  this->get_parameter("livox_cfg_path", livox_cfg_path);

  RCLCPP_INFO(this->get_logger(), "Parametro autostart = %s", autostart ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "Parametro livox_cfg_path = %s", livox_cfg_path.c_str());


  // 3. Inizializzazione SDK con livox_cfg_path (esempio)
  if (!livox_cfg_path.empty()) {
    // qui chiami l’SDK passando il JSON config
    // es: LivoxLidarSdkInit(livox_cfg_path.c_str());
    RCLCPP_INFO(this->get_logger(), "SDK inizializzato con config: %s", livox_cfg_path.c_str());
  } else {
    RCLCPP_WARN(this->get_logger(), "Nessun livox_cfg_path specificato!");
  }

  this -> livox_cfg_path_=livox_cfg_path;
  this -> autostart_=autostart;
  //FINE MODIFICA



  // Creazione del servizio /msg/Mid360_Server, legato al callback Mid360_Enable_Disable_clbk
  server_ = this->create_service<MsgEnableDisable>(
    "/msg/Mid360_Server",
    std::bind(
      &Mid360_Server::Mid360_Enable_Disable_clbk,  // Metodo da chiamare alle richieste
      this,
      std::placeholders::_1,
      std::placeholders::_2));                     // Placeholder per il parametro request

  RCLCPP_INFO(this->get_logger(), "Server initialized");  // Log di avvenuta inizializzazione

  //  Publisher PointCloud2
  pc2_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/msg_MID360/PointCloud2", rclcpp::QoS(10));
  RCLCPP_INFO(this->get_logger(), "Publisher PointCloud2 avviato");

  //  Publisher IMU
  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
    "/msg_MID360/IMU", rclcpp::QoS(10));
  RCLCPP_INFO(this->get_logger(), "Publisher IMU avviato");

  if(autostart){
    Mid360_autostart();
  }




}