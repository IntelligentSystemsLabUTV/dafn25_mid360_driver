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
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <string>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "livox_lidar_def.h"  // contiene LivoxLidarKeyValueParam

void Mid360_Server::on_pub_timer() {
  // Se non ci sono punti o il puntatore è nullo, non pubblichiamo
  //printf("\n**************     PC2 PUB     *****************\n");
  if (!this->frame_ready_ || this->last_point == nullptr || this->num_punti == 0)
    return;

  sensor_msgs::msg::PointCloud2 pc2_msg;

  // Header
  pc2_msg.header.stamp = this->now();
  pc2_msg.header.frame_id = "map";
  pc2_msg.height = 1;
  pc2_msg.width  = this->num_punti;

  // Definizione dei campi
  pc2_msg.fields.resize(5);

  pc2_msg.fields[0].name = "x";
  pc2_msg.fields[0].offset = 0;
  pc2_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  pc2_msg.fields[0].count = 1;

  pc2_msg.fields[1].name = "y";
  pc2_msg.fields[1].offset = 4;
  pc2_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  pc2_msg.fields[1].count = 1;

  pc2_msg.fields[2].name = "z";
  pc2_msg.fields[2].offset = 8;
  pc2_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  pc2_msg.fields[2].count = 1;

  pc2_msg.fields[3].name = "reflectivity";
  pc2_msg.fields[3].offset = 12;
  pc2_msg.fields[3].datatype = sensor_msgs::msg::PointField::UINT8;
  pc2_msg.fields[3].count = 1;

  pc2_msg.fields[4].name = "tag";
  pc2_msg.fields[4].offset = 13;
  pc2_msg.fields[4].datatype = sensor_msgs::msg::PointField::UINT8;
  pc2_msg.fields[4].count = 1;

  // Layout PointCloud2
  pc2_msg.is_bigendian = false;
  pc2_msg.point_step   = 3 * sizeof(float) + 2 * sizeof(uint8_t); // 14 bytes
  pc2_msg.row_step     = pc2_msg.point_step * pc2_msg.width;
  pc2_msg.is_dense     = true;

  pc2_msg.data.resize(pc2_msg.row_step);

  // Riempimento dei dati
  for (size_t i = 0; i < this->num_punti; ++i) {
    LivoxLidarCartesianHighRawPoint p = this->last_point[i];

    float x = p.x * 0.001f; // mm → m
    float y = p.y * 0.001f;
    float z = p.z * 0.001f;

    uint8_t reflectivity = p.reflectivity;
    uint8_t tag = p.tag;

    uint8_t* ptr = &pc2_msg.data[i * pc2_msg.point_step];

    memcpy(ptr + 0, &x, sizeof(float));
    memcpy(ptr + 4, &y, sizeof(float));
    memcpy(ptr + 8, &z, sizeof(float));
    memcpy(ptr + 12, &reflectivity, sizeof(uint8_t));
    memcpy(ptr + 13, &tag, sizeof(uint8_t));
  }

  //printf("\n**************     PC2 SEND     *****************\n");
  // Pubblica il messaggio
  pc2_pub_->publish(pc2_msg);
  this->frame_ready_ = false;
}


void Mid360_Server::on_pub_timer_IMU()
{
  //printf("\n**************     IMU SEND     *****************\n");
  float gyro_x = this->gyro[0];
  float gyro_y = this->gyro[1];
  float gyro_z = this->gyro[2];
  float acc_x  = this->acc[0];
  float acc_y  = this->acc[1];
  float acc_z  = this->acc[2];

  sensor_msgs::msg::Imu imu_msg;

  // Header
  imu_msg.header.stamp = this->now();
  imu_msg.header.frame_id = "imu_link";

  // Angular velocity (gyroscope)
  imu_msg.angular_velocity.x = gyro_x;
  imu_msg.angular_velocity.y = gyro_y;
  imu_msg.angular_velocity.z = gyro_z;

  // Linear acceleration (accelerometer)
  imu_msg.linear_acceleration.x = acc_x;
  imu_msg.linear_acceleration.y = acc_y;
  imu_msg.linear_acceleration.z = acc_z;

  //pubblicazione
  imu_pub_->publish(imu_msg);



}