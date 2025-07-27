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


static constexpr int PUB_PERIOD_MS =1;
static constexpr int PUB_PERIOD_MS_IMU =1000;
static constexpr int PUB_PERIOD_MS_INFO =1000;

//aggiungo 
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <string>

/*
void QueryInternalInfoCallback(livox_status status, uint32_t handle, 
    LivoxLidarDiagInternalInfoResponse* response, void* client_data) {
      printf("\n\n\n Query: ");
  if (status != kLivoxLidarStatusSuccess) {
    printf("Query lidar internal info failed.\n");
    QueryLivoxLidarInternalInfo(handle, QueryInternalInfoCallback, nullptr);
    return;
  }

  if (response == nullptr) {
    return;
  }

  uint8_t host_point_ipaddr[4] {0};
  uint16_t host_point_port = 0;
  uint16_t lidar_point_port = 0;

  uint8_t host_imu_ipaddr[4] {0};
  uint16_t host_imu_data_port = 0;
  uint16_t lidar_imu_data_port = 0;

  uint16_t off = 0;
  for (uint8_t i = 0; i < response->param_num; ++i) {
    LivoxLidarKeyValueParam* kv = (LivoxLidarKeyValueParam*)&response->data[off];
    if (kv->key == kKeyLidarPointDataHostIpCfg) {
      memcpy(host_point_ipaddr, &(kv->value[0]), sizeof(uint8_t) * 4);
      memcpy(&(host_point_port), &(kv->value[4]), sizeof(uint16_t));
      memcpy(&(lidar_point_port), &(kv->value[6]), sizeof(uint16_t));
    } else if (kv->key == kKeyLidarImuHostIpCfg) {
      memcpy(host_imu_ipaddr, &(kv->value[0]), sizeof(uint8_t) * 4);
      memcpy(&(host_imu_data_port), &(kv->value[4]), sizeof(uint16_t));
      memcpy(&(lidar_imu_data_port), &(kv->value[6]), sizeof(uint16_t));
    }
    off += sizeof(uint16_t) * 2;
    off += kv->length;
  }

  printf("Host point cloud ip addr:%u.%u.%u.%u, host point cloud port:%u, lidar point cloud port:%u.\n",
      host_point_ipaddr[0], host_point_ipaddr[1], host_point_ipaddr[2], host_point_ipaddr[3], host_point_port, lidar_point_port);

  printf("Host imu ip addr:%u.%u.%u.%u, host imu port:%u, lidar imu port:%u.\n",
    host_imu_ipaddr[0], host_imu_ipaddr[1], host_imu_ipaddr[2], host_imu_ipaddr[3], host_imu_data_port, lidar_imu_data_port);

}*/
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "livox_lidar_def.h"  // contiene LivoxLidarKeyValueParam


const char* DeviceTypeToString(LivoxLidarDeviceType type) {
  switch (type) {
    case kLivoxLidarTypeHub: return "Hub";
    case kLivoxLidarTypeMid40: return "Mid-40";
    case kLivoxLidarTypeTele: return "Tele";
    case kLivoxLidarTypeHorizon: return "Horizon";
    case kLivoxLidarTypeMid70: return "Mid-70";
    case kLivoxLidarTypeAvia: return "Avia";
    case kLivoxLidarTypeMid360: return "Mid-360";
    case kLivoxLidarTypeIndustrialHAP: return "Industrial HAP";
    case kLivoxLidarTypeHAP: return "HAP";
    case kLivoxLidarTypePA: return "PA";
    default: return "Unknown DeviceType";
  }
}

// Funzione helper: converte la chiave in stringa
const char* ParamKeyToString(uint16_t key) {
  switch (key) {
    case kKeyPclDataType: return "kKeyPclDataType";
    case kKeyPatternMode: return "kKeyPatternMode";
    case kKeyDualEmitEn: return "kKeyDualEmitEn";
    case kKeyPointSendEn: return "kKeyPointSendEn";
    case kKeyLidarIpCfg: return "kKeyLidarIpCfg";
    case kKeyStateInfoHostIpCfg: return "kKeyStateInfoHostIpCfg";
    case kKeyLidarPointDataHostIpCfg: return "kKeyLidarPointDataHostIpCfg";
    case kKeyLidarImuHostIpCfg: return "kKeyLidarImuHostIpCfg";
    case kKeyCtlHostIpCfg: return "kKeyCtlHostIpCfg";
    case kKeyLogHostIpCfg: return "kKeyLogHostIpCfg";
    case kKeyVehicleSpeed: return "kKeyVehicleSpeed";
    case kKeyEnvironmentTemp: return "kKeyEnvironmentTemp";
    case kKeyInstallAttitude: return "kKeyInstallAttitude";
    case kKeyBlindSpotSet: return "kKeyBlindSpotSet";
    case kKeyFrameRate: return "kKeyFrameRate";
    case kKeyFovCfg0: return "kKeyFovCfg0";
    case kKeyFovCfg1: return "kKeyFovCfg1";
    case kKeyFovCfgEn: return "kKeyFovCfgEn";
    case kKeyDetectMode: return "kKeyDetectMode";
    case kKeyFuncIoCfg: return "kKeyFuncIoCfg";
    case kKeyWorkModeAfterBoot: return "kKeyWorkModeAfterBoot";
    case kKeyWorkMode: return "kKeyWorkMode";
    case kKeyGlassHeat: return "kKeyGlassHeat";
    case kKeyImuDataEn: return "kKeyImuDataEn";
    case kKeyFusaEn: return "kKeyFusaEn";
    case kKeyForceHeatEn: return "kKeyForceHeatEn";
    case kKeySn: return "kKeySn";
    case kKeyProductInfo: return "kKeyProductInfo";
    case kKeyVersionApp: return "kKeyVersionApp";
    case kKeyVersionLoader: return "kKeyVersionLoader";
    case kKeyVersionHardware: return "kKeyVersionHardware";
    case kKeyMac: return "kKeyMac";
    case kKeyCurWorkState: return "kKeyCurWorkState";
    case kKeyCoreTemp: return "kKeyCoreTemp";
    case kKeyPowerUpCnt: return "kKeyPowerUpCnt";
    case kKeyLocalTimeNow: return "kKeyLocalTimeNow";
    case kKeyLastSyncTime: return "kKeyLastSyncTime";
    case kKeyTimeOffset: return "kKeyTimeOffset";
    case kKeyTimeSyncType: return "kKeyTimeSyncType";
    case kKeyStatusCode: return "kKeyStatusCode";
    case kKeyLidarDiagStatus: return "kKeyLidarDiagStatus";
    case kKeyLidarFlashStatus: return "kKeyLidarFlashStatus";
    case kKeyFwType: return "kKeyFwType";
    case kKeyHmsCode: return "kKeyHmsCode";
    case kKeyCurGlassHeatState: return "kKeyCurGlassHeatState";
    default: return "Unknown Key";
  }
}
void QueryFwTypeCallback(livox_status status, uint32_t handle,
                         LivoxLidarDiagInternalInfoResponse* response, void* client_data) {
  if (status != kLivoxLidarStatusSuccess || response == nullptr) {
    printf("QueryFwTypeCallback: errore o nessuna risposta.\n");
    return;
  }

  if (response->param_num == 0) {
    printf("QueryFwTypeCallback: nessun parametro restituito.\n");
    return;
  }

  // Il tipo è nel primo byte della risposta
  LivoxLidarDeviceType type = static_cast<LivoxLidarDeviceType>(response->data[0]);

  printf("Device Type: %s (%d)\n", DeviceTypeToString(type), type);
}

void QueryFwVersionCallback(livox_status status, uint32_t handle,
                            LivoxLidarDiagInternalInfoResponse* response, void* client_data) {
  if (status != kLivoxLidarStatusSuccess || response == nullptr) return;

  if (response->param_num > 0) {
    uint8_t major = response->data[0];
    uint8_t minor = response->data[1];
    uint8_t patch = response->data[2];

    printf("Firmware Version: %d.%d.%d\n", major, minor, patch);
  }
}

void QueryInternalInfoCallback(livox_status status, uint32_t handle,
                               LivoxLidarDiagInternalInfoResponse* response, void* client_data) {
  if (status != kLivoxLidarStatusSuccess || response == nullptr) return;

  uint16_t off = 0;
  for (uint8_t i = 0; i < response->param_num; ++i) {
    LivoxLidarKeyValueParam* kv = (LivoxLidarKeyValueParam*)&response->data[off];

    printf("Key: %s (0x%04X), Length: %u, Value: ",
           ParamKeyToString(kv->key), kv->key, kv->length);

    // Stampa il contenuto in esadecimale
    for (int j = 0; j < kv->length; j++) {
      printf("%02X ", kv->value[j]);
    }
    printf("\n");

    off += sizeof(uint16_t) * 2 + kv->length;
  }
}


void WorkModeCallback(livox_status status, uint32_t handle,LivoxLidarAsyncControlResponse *response, void *client_data) {
  printf("\n\n\n *** Sei entrato in work mode *** \n");
  if (response == nullptr) {
    return;
  }
  printf("WorkModeCallack, status:%u, handle:%u, ret_code:%u, error_key:%u \n\n\n",
      status, handle, response->ret_code, response->error_key);
}

void LoggerStartCallback(livox_status status, uint32_t handle, LivoxLidarLoggerResponse* response, void* client_data) {
  if (status != kLivoxLidarStatusSuccess) {
    printf("\n\n\n Start logger failed, the status :%d\n\n\n", status);
    LivoxLidarStartLogger(handle,  kLivoxLidarRealTimeLog, LoggerStartCallback, nullptr);
    return;
  }

  if (response == nullptr) {
    printf("\n\n\n Start logger failed, the response is nullptr.\n\n\n");
    LivoxLidarStartLogger(handle,  kLivoxLidarRealTimeLog, LoggerStartCallback, nullptr);
    return;
  }

  if (response->ret_code != 0) {
    printf("\n\n\n Start logger failed, the response ret_Code:%d.\n\n\n", response->ret_code);
    LivoxLidarStartLogger(handle,  kLivoxLidarRealTimeLog, LoggerStartCallback, nullptr);
    return;
  }

  printf("The lidar[%u] start logger succ.\n", handle);
}
void PushMsgCallback(const uint32_t handle, const uint8_t dev_type, const char* info, void* client_data) {
  printf("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA\n");
  auto self = static_cast<Mid360_Server*>(client_data);
  struct in_addr tmp_addr;
  tmp_addr.s_addr = handle;  
  printf("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA\n");
  printf("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA\n");

  //copio dati struct 
  self->info_addr = tmp_addr;
  self->info_livox = info;
  return;
}

void LivoxLidarAsyncControlClbk(livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse *response, void *client_data)
{
  printf("\n\nPrintf in LivoxLidarAsync status: %d ret_code: %d \n\n",status, response->ret_code);
  return;
}

void LidarInfoChangeCallback(const uint32_t handle, const LivoxLidarInfo* info, void* client_data) {
  printf("\n\n\n *** Sei entrato in lidar info change *** \n");
  if (info == nullptr) {
    printf("lidar info change callback failed, the info is nullptr.\n");
    return;
  } 
  printf("LidarInfoChangeCallback Lidar handle: %u SN: %s\n", handle, info->sn);
  SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, WorkModeCallback, nullptr);
  //logger abilitato. Stampa tante cose
  LivoxLidarStartLogger(handle, kLivoxLidarRealTimeLog, LoggerStartCallback, nullptr);
  QueryLivoxLidarInternalInfo(handle, QueryInternalInfoCallback, nullptr);
  QueryLivoxLidarFwType(handle, QueryFwTypeCallback, nullptr);
  QueryLivoxLidarFirmwareVer(handle, QueryFwVersionCallback, nullptr);
  SetLivoxLidarInfoCallback(PushMsgCallback, nullptr);

  LivoxLidarPointDataType pointDataType = kLivoxLidarCartesianCoordinateHighData;
  //LivoxLidarPointDataType pointDataType = kLivoxLidarSphericalCoordinateData;
  //LivoxLidarPointDataType pointDataType = kLivoxLidarCartesianCoordinateLowData;
  SetLivoxLidarPclDataType(handle, pointDataType , LivoxLidarAsyncControlClbk , nullptr);
  

}



void RebootCallback(livox_status status, uint32_t handle, LivoxLidarRebootResponse* response, void* client_data) {
  printf("RebootCallback, status:%u, handle:%u, ret_code:%u",
      status, handle, response->ret_code);
  return;
}



/*
void PointCloudCallback(uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data) {
  if (data == nullptr) {
    return;
  }
  auto self = static_cast<Mid360_Server*>(client_data);

  if(self->handlePc2 != handle){
    self->handlePc2 = handle;
    
  }


  
  
  //printf("\n\n HandlePc2 in Callback= %d \n\n",self->handlePc2);

  //printf("point cloud handle: %u, data_num: %d, data_type: %d, length: %d, frame_counter: %d\n",
  //   handle, data->dot_num, data->data_type, data->length, data->frame_cnt);

  self->num_punti = data->dot_num;

  if (data->data_type == kLivoxLidarCartesianCoordinateHighData) {
    LivoxLidarCartesianHighRawPoint *p_point_data = (LivoxLidarCartesianHighRawPoint *)data->data;
    for (uint32_t i = 0; i < data->dot_num; i++) {
      p_point_data[i].x;
      p_point_data[i].y;
      p_point_data[i].z;

     //printf("data[%d]: x:%d , y=%d , z=%d \n",i,p_point_data[i].x,p_point_data[i].y,p_point_data[i].z);
    }

    self->last_point = p_point_data;
  }
  else if (data->data_type == kLivoxLidarCartesianCoordinateLowData) {
    LivoxLidarCartesianLowRawPoint *p_point_data = (LivoxLidarCartesianLowRawPoint *)data->data;
    for (uint32_t i = 0; i < data->dot_num; i++) {
      p_point_data[i].x;
      p_point_data[i].y;
      p_point_data[i].z;

      //printf("data[%d]: x:%d , y=%d , z=%d \n",i,p_point_data[i].x,p_point_data[i].y,p_point_data[i].z);
    }


  } else if (data->data_type == kLivoxLidarSphericalCoordinateData) {
    LivoxLidarSpherPoint* p_point_data = (LivoxLidarSpherPoint *)data->data;

  }

}
*/
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

  // Se udp_cnt == 0 → inizia un nuovo frame → copio i dati e resetto il buffer
  if (data->udp_cnt == 0 && !self->frame_buffer_.empty()) {
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
    //formato dati
  /*
    p_point_data_imu[1].gyro_x;
    p_point_data_imu[1].gyro_y;
    p_point_data_imu[1].gyro_z;
    p_point_data_imu[1].acc_x;
    p_point_data_imu[1].acc_y;
    p_point_data_imu[1].acc_z;
  */

   //printf("\n\n");
   /*printf("data_imu: gyro_x:%f , gyro_y=%f , gyro_z=%f , acc_x:%f ,acc_y:%f ,acc_z:%f \n",
    p_point_data_imu[1].gyro_x,
    p_point_data_imu[1].gyro_y,
    p_point_data_imu[1].gyro_z,
    p_point_data_imu[1].acc_x,
    p_point_data_imu[1].acc_y,
    p_point_data_imu[1].acc_z);*/

   //printf("\n\n");
  
  auto self = static_cast<Mid360_Server*>(client_data);

  self->gyro = {p_point_data_imu[1].gyro_x,p_point_data_imu[1].gyro_y,p_point_data_imu[1].gyro_z};
  self->acc = {p_point_data_imu[1].acc_x,p_point_data_imu[1].acc_y,p_point_data_imu[1].acc_z};

}



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

  // 6) Timer per pubblicazione periodica (se ti serve)
  pub_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(PUB_PERIOD_MS),
    std::bind(&Mid360_Server::on_pub_timer, this));
  RCLCPP_INFO(this->get_logger(), "Timer di publish impostato a %d ms", PUB_PERIOD_MS);
  
  // 6) Timer per pubblicazione periodica (se ti serve)
  pub_timer_IMU_ = this->create_wall_timer(
    std::chrono::milliseconds(PUB_PERIOD_MS_IMU),
    std::bind(&Mid360_Server::on_pub_timer_IMU, this));
  RCLCPP_INFO(this->get_logger(), "Timer di publish IMU impostato a %d ms", PUB_PERIOD_MS_IMU);

  // 6) Timer per pubblicazione periodica (se ti serve)
  pub_timer_INFO_ = this->create_wall_timer(
    std::chrono::milliseconds(PUB_PERIOD_MS_INFO),
    std::bind(&Mid360_Server::on_pub_timer_INFO, this));
  RCLCPP_INFO(this->get_logger(), "Timer di publish INFOimpostato a %d ms", PUB_PERIOD_MS_INFO);
}

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
    
    
   //INFO (Serve che il livox mandi un msg non so come )
   // SetLivoxLidarInfoCallback(PushMsgCallback, this);
    // pointCloud
    SetLivoxLidarPointCloudCallBack(PointCloudCallback, this);
    // IMU
    SetLivoxLidarImuDataCallback(ImuDataCallback, this);
    //version
    this->Version = new LivoxLidarSdkVer;
    GetLivoxLidarSdkVer(this->Version);
    //DeviceType
    LivoxLidarDeviceType DeviceType = kLivoxLidarTypeMid360;
    //PointDataType 
    LivoxLidarPointDataType pointDataType;
    livox_status statusDataType;


    //printf("\n\n HandlePc2 = %d \n\n",this->handlePc2);
    //statusDataType = SetLivoxLidarPclDataType(this->handlePc2, pointDataType , LivoxLidarAsyncControlClbk,this);
    
    //printf("\n\nLivox Status = %d\n\n",statusDataType);
    //printf("\n\nDeviceType kLivoxLidarTypeMid360= %d\n\n", DeviceType);
    //printf("\n\nVersione SDK-> major:%d  minor:%d   patch:%d\n\n", this->Version->major, this->Version->minor, this->Version->patch);

    //info change
    SetLivoxLidarInfoChangeCallback(LidarInfoChangeCallback, this);

    //distruttore 
    if (this->Version) {
    delete this->Version;
    this->Version = nullptr;
    }

    response->success = true;        // Segnala successo
  }
  else if (request->command == 0) {
    RCLCPP_INFO(get_logger(), "[Service] Disable"); // Log comando di DISABLE
    LivoxLidarSdkUninit();                          // Disattiva il driver Livox
    response->success = true;                       // Segnala successo
  }
  else if(request->command == 10){
    RCLCPP_INFO(get_logger(), "[Service] Reboot"); // Log comando di reboot

    // invia il reboot
    RCLCPP_INFO(get_logger(), "[Service] Rebooting lidar handle=%u", handlePc2);
    LivoxLidarRequestReboot(handlePc2, RebootCallback, this);
    response->success = true;                       // Segnala successo

  }

  else {
    RCLCPP_WARN(get_logger(), "[Service] Comando non valido: %d", request->command);
    response->success = false;                      // Segnala fallimento
  }
}

void Mid360_Server::on_pub_timer() {
  // Se non ci sono punti o il puntatore è nullo, non pubblichiamo
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

  // Pubblica il messaggio
  pc2_pub_->publish(pc2_msg);
  this->frame_ready_ = false;
}
/*
void Mid360_Server::on_pub_timer() {
  // Se non ci sono punti o il puntatore è nullo, non pubblichiamo
  if (this->last_point == nullptr || this->num_punti == 0)
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

  // Pubblica il messaggio
  pc2_pub_->publish(pc2_msg);
}
*/

/*
void Mid360_Server::on_pub_timer()
{
  std::vector<int32_t> x_vals{0};
  std::vector<int32_t> y_vals{0};
  std::vector<int32_t> z_vals{0};
  std::vector<uint8_t> reflectivity_vals{0};
  std::vector<uint8_t> tag_vals{0};
  sensor_msgs::msg::PointCloud2 pc2_msg;

  //1) campi
  pc2_msg.header.stamp = this->now();
  pc2_msg.header.frame_id = "map";
  pc2_msg.height = 1;
  pc2_msg.width  = this->num_punti;

  //printf("\n**************dot_num PC2 = %u*****************\n",this->num_punti);

  // 1) Preparo esattamente 5 campi
  pc2_msg.fields.resize(5);

  // 2) Imposto manualmente ciascun elemento
  pc2_msg.fields[0].name     = "x";
  pc2_msg.fields[0].offset   = 0;
  pc2_msg.fields[0].datatype = sensor_msgs::msg::PointField::INT32;
  pc2_msg.fields[0].count    = 1;

  pc2_msg.fields[1].name     = "y";
  pc2_msg.fields[1].offset   = 4;
  pc2_msg.fields[1].datatype = sensor_msgs::msg::PointField::INT32;
  pc2_msg.fields[1].count    = 1;

  pc2_msg.fields[2].name     = "z";
  pc2_msg.fields[2].offset   = 8;
  pc2_msg.fields[2].datatype = sensor_msgs::msg::PointField::INT32;
  pc2_msg.fields[2].count    = 1;

  pc2_msg.fields[3].name     = "reflectivity";
  pc2_msg.fields[3].offset   = 12;
  pc2_msg.fields[3].datatype = sensor_msgs::msg::PointField::UINT8;
  pc2_msg.fields[3].count    = 1;

  pc2_msg.fields[4].name     = "tag";
  pc2_msg.fields[4].offset   = 13;
  pc2_msg.fields[4].datatype = sensor_msgs::msg::PointField::UINT8;
  pc2_msg.fields[4].count    = 1;


  // 3) parametri layout
  pc2_msg.is_bigendian = false;
  pc2_msg.point_step   = sizeof(int32_t)*3 + sizeof(uint8_t)*2; // 14
  pc2_msg.row_step     = pc2_msg.point_step * pc2_msg.width;   // 14 * 96
  pc2_msg.is_dense     = true;

  pc2_msg.data.resize(pc2_msg.row_step);

  // 4) Riempio gli elementi 
  for (size_t i = 0; i < pc2_msg.width; ++i) {
    uint8_t* ptr = &pc2_msg.data[i * pc2_msg.point_step];

    memcpy(ptr + 0,  &x_vals[i], sizeof(int32_t));
    memcpy(ptr + 4,  &y_vals[i], sizeof(int32_t));
    memcpy(ptr + 8,  &z_vals[i], sizeof(int32_t));
    memcpy(ptr + 12, &reflectivity_vals[i], sizeof(uint8_t));
    memcpy(ptr + 13, &tag_vals[i], sizeof(uint8_t));
  }

  //Pubblicazione
  pc2_pub_->publish(pc2_msg);

}*/


void Mid360_Server::on_pub_timer_IMU()
{
  //printf("\n**************     IMU     *****************\n");
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



void Mid360_Server::on_pub_timer_INFO()
{

  //printf("\n**************     INFO     *****************\n");
  livox_lidar_interfaces::msg::LivoxInfo info_msg;



  info_msg.serial_number = this->info_livox ? 
                         std::string(this->info_livox) : 
                         "";

  info_msg.lidar_ip_address = (this->info_addr.s_addr != 0) ?
                            std::string(inet_ntoa(this->info_addr)) : "";


  //pubblicazione
  info_pub_->publish(info_msg);

}


int main(int argc, char ** argv)
{
  std::cout << ">>> SERVER MAIN() STARTED <<<\n";
  rclcpp::init(argc, argv);                     // Inizializza il sistema ROS2
  auto server_node = std::make_shared<Mid360_Server>(); // Crea il nodo server
  rclcpp::spin(server_node);                     // Mantiene vivo il nodo per gestire le richieste
  rclcpp::shutdown();                            // Chiude il sistema ROS2
  exit(EXIT_SUCCESS);                            // Esce con codice 0 (successo)
}
