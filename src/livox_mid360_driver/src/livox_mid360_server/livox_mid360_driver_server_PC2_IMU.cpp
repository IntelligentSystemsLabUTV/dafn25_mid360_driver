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

  if (self->handlePc2 != handle) {
    self->handlePc2 = handle;
  }

  if (data->data_type != kLivoxLidarCartesianCoordinateHighData) {
    return;
  }

  auto* p_point_data = reinterpret_cast<LivoxLidarCartesianHighRawPoint*>(data->data);

  // Aggiunge i punti del pacchetto al buffer
  self->frame_buffer_.insert(
      self->frame_buffer_.end(),
      p_point_data,
      p_point_data + data->dot_num);

  // Se udp_cnt == 0 → inizia un nuovo frame → copio i dati e resetto il buffer (non Funziona, usiamo il modulo)
  if ((data->udp_cnt % 209) == 0 && !self->frame_buffer_.empty()) {
    self->num_punti = self->frame_buffer_.size();

    delete[] self->last_point;
    self->last_point = new LivoxLidarCartesianHighRawPoint[self->num_punti];

    memcpy(self->last_point, self->frame_buffer_.data(),
           self->num_punti * sizeof(LivoxLidarCartesianHighRawPoint));

    self->frame_ready_ = true;

    // reset buffer per il frame successivo
    self->frame_buffer_.clear();
  }
}


void ImuDataCallback(uint32_t handle, const uint8_t dev_type,  LivoxLidarEthernetPacket* data, void* client_data) {
  if (data == nullptr) {
    return;
  } 
 // printf("Imu data callback handle:%u, data_num:%u, data_type:%u, length:%u, frame_counter:%u.\n",
 //   handle, data->dot_num, data->data_type, data->length, data->frame_cnt);

  LivoxLidarImuRawPoint *p_point_data_imu = (LivoxLidarImuRawPoint *)data->data;
   
  
  auto self = static_cast<Mid360_Server*>(client_data);

  self->gyro = {p_point_data_imu[1].gyro_x,p_point_data_imu[1].gyro_y,p_point_data_imu[1].gyro_z};
  self->acc = {p_point_data_imu[1].acc_x,p_point_data_imu[1].acc_y,p_point_data_imu[1].acc_z};

}