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




void Mid360_Client::infoCallback(
  const livox_lidar_interfaces::msg::LivoxInfo::SharedPtr msg)
{
  RCLCPP_DEBUG(this->get_logger(), "Ricevuto LivoxInfo, model=");
  RCLCPP_INFO(this->get_logger(), "******** STAMPA INFO ********");
  /*
  RCLCPP_INFO(this->get_logger(), 
    "serial number = %s , ip_address = %s",,
    msg->serial_number.c_str(),
    msg->lidar_ip_address.c_str());*/
    
  RCLCPP_INFO(this->get_logger(),
  "serial_number = %s, ip_address = %s, device_type = %d, point_data_type = %s, scan_pattern = %s, frame_rate = %d, work_mode = %s",
    msg->serial_number.c_str(),
    msg->lidar_ip_address.c_str(),
    msg->device_type,
    msg->point_data_type.c_str(),
    msg->scan_pattern.c_str(),
    msg->frame_rate,
    msg->work_mode.c_str()
  );


  // … processa msg …
}