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


void Mid360_Server::Mid360_Enable_Disable_clbk(
  const MsgEnableDisable::Request::SharedPtr request,   // Puntatore alla richiesta in arrivo
  const MsgEnableDisable::Response::SharedPtr response) // Puntatore alla risposta da riempire
{
  // Stampa il comando ricevuto:
  RCLCPP_INFO(get_logger(), "Received request->command = %d", request->command);



    // Ottieni la cartella share/<package>
  std::string pkg_share =
    ament_index_cpp::get_package_share_directory("livox_mid360_driver");

  // Componi il path assoluto al JSON
  std::string cfg_path = pkg_share + "/config/mid360_config.json";

  if (request->command == 1) {
    RCLCPP_INFO(get_logger(), "[Service] Enable");  // Log comando di ENABLE

    
    // Inizializza il driver Livox con il file di configurazione
    if (!LivoxLidarSdkInit(cfg_path.c_str())) {
      printf("Livox Init Failed\n");  // Stampa errore in console se fallisce
      LivoxLidarSdkUninit();         // Pulizia del driver
    }
    LivoxLidarSdkStart();
    

    // pointCloud
    SetLivoxLidarPointCloudCallBack(PointCloudCallback, this);
    // IMU
    SetLivoxLidarImuDataCallback(ImuDataCallback, this);
    //version
    this->Version = new LivoxLidarSdkVer;
    GetLivoxLidarSdkVer(this->Version);
    LivoxLidarSdkStart();

    response->success = true;        // Segnala successo

    //distruttore 
    if (this->Version) {
    delete this->Version;
    this->Version = nullptr;
    }
  }
  else if (request->command == 0) {
    RCLCPP_INFO(get_logger(), "[Service] Disable"); // Log comando di DISABLE
    LivoxLidarSdkUninit();                          // Disattiva il driver Livox
    response->success = true;                       // Segnala successo
  }
  else {

    RCLCPP_WARN(get_logger(), "[Service] Comando non valido: %d", request->command);
    response->success = false;                      // Segnala fallimento
  }
}






