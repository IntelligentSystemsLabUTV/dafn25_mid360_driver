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

//aggiungo 
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <string>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "livox_lidar_def.h"  // contiene LivoxLidarKeyValueParam


void PointCloudCallback(uint32_t handle, const uint8_t dev_type,
                        LivoxLidarEthernetPacket* data, void* client_data) {
  if (!data) return;
  auto self = static_cast<Mid360_Server*>(client_data);

  if (data->data_type != kLivoxLidarCartesianCoordinateHighData) {
    return;
  }

  auto* p_point_data =
    reinterpret_cast<LivoxLidarCartesianHighRawPoint*>(data->data);

  // Accumula i punti nel buffer
  self->frame_buffer_.insert(
      self->frame_buffer_.end(),
      p_point_data,
      p_point_data + data->dot_num);

  // Se udp_cnt % 209 == 0 → consideriamo questo "fine frame"
  if ((data->udp_cnt % 209) == 0 && !self->frame_buffer_.empty()) {
    self->num_punti = self->frame_buffer_.size();

    // Alloca spazio per i punti
    delete[] self->last_point;
    self->last_point = new LivoxLidarCartesianHighRawPoint[self->num_punti];
    memcpy(self->last_point, self->frame_buffer_.data(),
           self->num_punti * sizeof(LivoxLidarCartesianHighRawPoint));

    // --- Costruzione messaggio PointCloud2 ---
    sensor_msgs::msg::PointCloud2 pc2_msg;
    pc2_msg.header.stamp = self->now();
    pc2_msg.header.frame_id = "map";
    pc2_msg.height = 1;
    pc2_msg.width  = self->num_punti;

    pc2_msg.fields.resize(5);
    pc2_msg.fields[0].name = "x"; pc2_msg.fields[0].offset = 0;
    pc2_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pc2_msg.fields[0].count = 1;

    pc2_msg.fields[1].name = "y"; pc2_msg.fields[1].offset = 4;
    pc2_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pc2_msg.fields[1].count = 1;

    pc2_msg.fields[2].name = "z"; pc2_msg.fields[2].offset = 8;
    pc2_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pc2_msg.fields[2].count = 1;

    pc2_msg.fields[3].name = "reflectivity"; pc2_msg.fields[3].offset = 12;
    pc2_msg.fields[3].datatype = sensor_msgs::msg::PointField::UINT8;
    pc2_msg.fields[3].count = 1;

    pc2_msg.fields[4].name = "tag"; pc2_msg.fields[4].offset = 13;
    pc2_msg.fields[4].datatype = sensor_msgs::msg::PointField::UINT8;
    pc2_msg.fields[4].count = 1;

    pc2_msg.is_bigendian = false;
    pc2_msg.point_step   = 3 * sizeof(float) + 2 * sizeof(uint8_t); // 14 bytes
    pc2_msg.row_step     = pc2_msg.point_step * pc2_msg.width;
    pc2_msg.is_dense     = true;
    pc2_msg.data.resize(pc2_msg.row_step);

    for (size_t i = 0; i < self->num_punti; ++i) {
      LivoxLidarCartesianHighRawPoint p = self->last_point[i];
      float x = p.x * 0.001f;
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

    // Pubblica il messaggio
    self->pc2_pub_->publish(pc2_msg);

    // Svuota buffer per il frame successivo
    self->frame_buffer_.clear();
  }
}




void ImuDataCallback(uint32_t handle, const uint8_t dev_type,
                     LivoxLidarEthernetPacket* data, void* client_data) {
  if (!data) return;
  auto self = static_cast<Mid360_Server*>(client_data);

  LivoxLidarImuRawPoint* p_point_data_imu =
    reinterpret_cast<LivoxLidarImuRawPoint*>(data->data);

  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header.stamp = self->now();
  imu_msg.header.frame_id = "imu_link";

  imu_msg.angular_velocity.x = p_point_data_imu[1].gyro_x;
  imu_msg.angular_velocity.y = p_point_data_imu[1].gyro_y;
  imu_msg.angular_velocity.z = p_point_data_imu[1].gyro_z;

  imu_msg.linear_acceleration.x = p_point_data_imu[1].acc_x;
  imu_msg.linear_acceleration.y = p_point_data_imu[1].acc_y;
  imu_msg.linear_acceleration.z = p_point_data_imu[1].acc_z;

  // Pubblica direttamente
  self->imu_pub_->publish(imu_msg);
}
