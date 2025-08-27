

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



void Mid360_Client::pc2Callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg_PC2)
{

  
  RCLCPP_INFO(this->get_logger(), "Ricevuto PointCloud2 - frame: %s, dimensioni: %ux%u",
              msg_PC2->header.frame_id.c_str(), msg_PC2->height, msg_PC2->width);
  // stampa 
  RCLCPP_INFO(this->get_logger(),
              "Ricevuto PointCloud2: stamp=%u.%u frame_id=%s h=%u w=%u",
              msg_PC2->header.stamp.sec,
              msg_PC2->header.stamp.nanosec,
              msg_PC2->header.frame_id.c_str(),
              msg_PC2->height,
              msg_PC2->width
              );
  size_t N = std::size_t (msg_PC2->width);

  //RCLCPP_INFO(this->get_logger(), "******** STAMPA POINT CLOUD ********");

  for (size_t i = 0; i < N; ++i) {
    const uint8_t* ptr = &msg_PC2->data[i * msg_PC2->point_step];

    int32_t x, y, z;
    uint8_t reflectivity, tag;

    memcpy(&x, ptr + 0, sizeof(int32_t));
    memcpy(&y, ptr + 4, sizeof(int32_t));
    memcpy(&z, ptr + 8, sizeof(int32_t));
    memcpy(&reflectivity, ptr + 12, sizeof(uint8_t));
    memcpy(&tag, ptr + 13, sizeof(uint8_t));

    /*RCLCPP_INFO(this->get_logger(), 
      "[%zu] x: %d, y: %d, z: %d, reflectivity: %u, tag: %u", 
      i, x, y, z, reflectivity, tag);*/
  }
  

}

void Mid360_Client::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg_IMU)
{
   RCLCPP_INFO(this->get_logger(), "******** STAMPA IMU ********");
  RCLCPP_INFO(this->get_logger(), 
    "Angular Velocity -> x: %.3f, y: %.3f, z: %.3f",
    msg_IMU->angular_velocity.x,
    msg_IMU->angular_velocity.y,
    msg_IMU->angular_velocity.z);

  RCLCPP_INFO(this->get_logger(), 
    "Linear Acceleration -> x: %.3f, y: %.3f, z: %.3f",
    msg_IMU->linear_acceleration.x,
    msg_IMU->linear_acceleration.y,
    msg_IMU->linear_acceleration.z);
}



