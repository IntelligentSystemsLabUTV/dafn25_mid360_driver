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
static constexpr int PUB_PERIOD_MS_IMU =7000;
static constexpr int PUB_PERIOD_MS_INFO =11000;

//aggiungo 
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <string>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "livox_lidar_def.h"  // contiene LivoxLidarKeyValueParam


// Funzioni helper: converte la chiave in stringa
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


//Tipo del framework 
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

  auto self = static_cast<Mid360_Server*>(client_data);
  self->send_info_msg.device_type_str = DeviceTypeToString(type);
  self->send_info_msg.device_type_int = type;
  
  printf("\n\n****LIVOX_LIDAR_DEVICE_TYPE (inizio)****\n\n");
  //printf("Device Type: %s (%d)\n", DeviceTypeToString(type), type);
  printf("Device Type: %s (%d)\n", self->send_info_msg.device_type_str, self->send_info_msg.device_type_int);
  printf("\n\n****LIVOX_LIDAR_DEVICE_TYPE (fine)****\n\n");
  
}

//versione del framework
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
//codifica parametri key

void DecodeParam(const LivoxLidarKeyValueParam* kv,void* client_data) {
  printf("\nKey: %s (0x%04X)\n", ParamKeyToString(kv->key), kv->key);

  switch (kv->key) {
    case 0x8000: // Serial Number
    case 0x8001: // Product Info
      printf("Value (string): %.*s\n", kv->length, kv->value);
      break;

    case 0x8002: // App Version
    case 0x8003: // Loader Version
    case 0x8004: // Hardware Version
      printf("Version: %u.%u.%u.%u\n",
             kv->value[0], kv->value[1], kv->value[2], kv->value[3]);
      break;

    case 0x0004: // Lidar IP Config
    {
      const uint8_t* v = kv->value;
      printf("IP: %u.%u.%u.%u  Mask: %u.%u.%u.%u  GW: %u.%u.%u.%u\n",
             v[0], v[1], v[2], v[3],
             v[4], v[5], v[6], v[7],
             v[8], v[9], v[10], v[11]);
      break;
    }

    case 0x0005:
    case 0x0006:
    case 0x0007:
    {
      const uint8_t* v = kv->value;
      printf("IP: %u.%u.%u.%u  Port1: 0x%02X%02X  Port2: 0x%02X%02X\n",
             v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7]);
      break;
    }

    case 0x8005: // MAC
      printf("MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
             kv->value[0], kv->value[1], kv->value[2],
             kv->value[3], kv->value[4], kv->value[5]);
      break;

    case 0x8007: // Temperatura
    {
      uint32_t raw;
      memcpy(&raw, kv->value, sizeof(uint32_t));
      float temp = raw / 256.0f;
      printf("Temperature: %.2f °C\n", temp);
      break;
    }

    case 0x8008: // Power-Up count
    {
      uint32_t cnt;
      memcpy(&cnt, kv->value, sizeof(uint32_t));
      printf("Power-Up Count: %u\n", cnt);
      break;
    }

    case 0x8009:
    case 0x800A:
    case 0x800B:
    {
      uint64_t ts;
      memcpy(&ts, kv->value, sizeof(uint64_t));
      printf("Timestamp: %llu\n", (unsigned long long)ts);
      break;
    }

    case 0x0017: case 0x0018: case 0x001A:
    case 0x001C: case 0x8006: case 0x800C:
    case 0x8010:
      printf("Value: %u\n", kv->value[0]);
      break;

    default:
      printf("Value (hex): ");
      for (int j = 0; j < kv->length; j++) printf("%02X ", kv->value[j]);
      printf("\n");
      break;
  }

  // DATI RICHIESTI

  LivoxLidarSdkVer version;
  GetLivoxLidarSdkVer(&version);

  auto self = static_cast<Mid360_Server*>(client_data);

  self->send_info_msg.sdk_version = version;

   printf("\n\n****LIVOX_LIDAR_SDK_VERSIONE (inizio)****\n\n");
   printf("major = %d   minor = %d   patch = %d\n",self->send_info_msg.sdk_version.major,self->send_info_msg.sdk_version.minor,self->send_info_msg.sdk_version.patch);
   printf("\n\n****LIVOX_LIDAR_SDK_VERSIONE (fine)****\n\n");
}
//Informazioni interne
void QueryInternalInfoCallback(livox_status status, uint32_t handle,
                               LivoxLidarDiagInternalInfoResponse* response, void* client_data) {

  auto self = static_cast<Mid360_Server*>(client_data);

  if (status != kLivoxLidarStatusSuccess || response == nullptr) return;

  uint16_t off = 0;
  for (uint8_t i = 0; i < response->param_num; ++i) {
    auto* kv = reinterpret_cast<LivoxLidarKeyValueParam*>(&response->data[off]);
    DecodeParam(kv,self);  // Richiama la funzione di decodifica
    off += sizeof(uint16_t) * 2 + kv->length;
  }
}

/*
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
*/


void WorkModeCallback(livox_status status, uint32_t handle,LivoxLidarAsyncControlResponse *response, void *client_data) {
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

//Non abbiamo capito come funziona l'api che la richiama (Forse si attiva solo con pacchetti specifici)
void PushMsgCallback(const uint32_t handle, const uint8_t dev_type, const char* info, void* client_data) {
  auto self = static_cast<Mid360_Server*>(client_data);
  struct in_addr tmp_addr;
  tmp_addr.s_addr = handle;  

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


/*
void prova1(const uint32_t handle, const LivoxLidarInfo* info, void* client_data) {


  LivoxLidarWorkMode work_mode = kLivoxLidarNormal ;
  SetLivoxLidarWorkMode(handle, work_mode, nullptr, client_data);

}*/

void LidarInfoChangeCallback(const uint32_t handle, const LivoxLidarInfo* info, void* client_data) {
  if (info == nullptr) {
    printf("lidar info change callback failed, the info is nullptr.\n");
    return;
  } 
  printf("LidarInfoChangeCallback Lidar handle: %u SN: %s\n", handle, info->sn); //stampa handle e numero di serie
  
   auto self = static_cast<Mid360_Server*>(client_data);
  

  // Imposta il LiDAR in modalità di lavoro normale
  SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, WorkModeCallback, self);
  
  //logger disabilitato.
  //LivoxLidarStartLogger(handle, kLivoxLidarRealTimeLog, LoggerStartCallback, self);

  //Richiede parametri di configurazione 
  QueryLivoxLidarInternalInfo(handle, QueryInternalInfoCallback, self);
  
  // Richiede il tipo di firmware 
  QueryLivoxLidarFwType(handle, QueryFwTypeCallback, self);

  // Richiede la versione di firmware 
  QueryLivoxLidarFirmwareVer(handle, QueryFwVersionCallback, self);
  
  // Imposta la callback PushMsgCallback per ricevere messaggi dal LiDAR
  SetLivoxLidarInfoCallback(PushMsgCallback, self);
  
  // DATI PER INFO MSG

  self->send_info_msg.lidar_info = *info;
  
  // LivoxDataInfo
  printf("\n\n****LIVOX_DATA_INFO (inizio)****\n\n");
  printf("dev_type=%d\n",self->send_info_msg.lidar_info.dev_type);
  printf("serial_number=%s\n",self->send_info_msg.lidar_info.sn);
  printf("lidar_ip_address= %s\n",self->send_info_msg.lidar_info.lidar_ip);
  printf("\n\n****LIVOX_DATA_INFO (fine)****\n\n");
  
  // point data type
  LivoxLidarPointDataType pointDataTypeInfo = kLivoxLidarCartesianCoordinateHighData;
  self->send_info_msg.point_data_type = pointDataTypeInfo;
  printf("\n\n****LIVOX_LIDAR_POINT_DATA_TYPE (inizio)****\n\n");
  printf("Point Data Type (valore numerico): %d\n", (int)self->send_info_msg.point_data_type);
  printf("\n\n****LIVOX_LIDAR_POINT_DATA_TYPE (fine)****\n\n");

  //(Forse) setta il modo
  LivoxLidarPointDataType pointDataType = kLivoxLidarCartesianCoordinateHighData;
  //LivoxLidarPointDataType pointDataType = kLivoxLidarSphericalCoordinateData;
  //LivoxLidarPointDataType pointDataType = kLivoxLidarCartesianCoordinateLowData;
  //SetLivoxLidarPclDataType(handle, pointDataType , LivoxLidarAsyncControlClbk , nullptr);
  
  // scan pattern
  LivoxLidarScanPattern scanPatternInfo = kLivoxLidarScanPatternNoneRepetive;
  self->send_info_msg.scan_pattern = scanPatternInfo;
  printf("\n\n****LIVOX_LIDAR_SCAN_PATTERN (inizio)****\n\n");
  printf("Scan Pattern (valore numerico): %d\n", (int)self->send_info_msg.scan_pattern);
  printf("\n\n****LIVOX_LIDAR_SCAN_PATTERN (fine)****\n\n");
  
  LivoxLidarPointFrameRate pointFrameRateInfo = kLivoxLidarFrameRate10Hz;
  self->send_info_msg.point_frame_rate = pointFrameRateInfo;
  printf("\n\n****LIVOX_LIDAR_FRAME_RATE (inizio)****\n\n");
  printf("Frame Rate (valore numerico): %d\n", (int)self->send_info_msg.point_frame_rate);
  printf("\n\n****LIVOX_LIDAR_FRAME_RATE (fine)****\n\n");
  
  LivoxLidarWorkMode workModeInfo = kLivoxLidarNormal;
  self->send_info_msg.work_mode = workModeInfo;
  printf("\n\n****LIVOX_LIDAR_WORK_MODE (inizio)****\n\n");
  printf("Work Mode (valore numerico): %d\n", (int)self->send_info_msg.work_mode);
  printf("\n\n****LIVOX_LIDAR_WORK_MODE (fine)****\n\n");
}



void RebootCallback(livox_status status, uint32_t handle, LivoxLidarRebootResponse* response, void* client_data) {
  printf("RebootCallback, status:%u, handle:%u, ret_code:%u",
      status, handle, response->ret_code);
  return;
}

void prova(const uint32_t handle, const LivoxLidarInfo* info, void* client_data) {

  LivoxLidarRequestReboot(handle, RebootCallback, client_data);
}


void PointCloudCallback(uint32_t handle, const uint8_t dev_type,
                        LivoxLidarEthernetPacket* data, void* client_data) {
  //printf("\n\nSONO ENTRATO IN PC2\n\n");
  if (!data) return;
  //printf("\n\nSONO IN PC2 -> \n\n");

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
    //LivoxLidarSdkStart();
    

    //INFO (Serve che il livox mandi un msg non so come )
    //SetLivoxLidarInfoCallback(PushMsgCallback, this);
    // pointCloud
    SetLivoxLidarPointCloudCallBack(PointCloudCallback, this);
    // IMU
    SetLivoxLidarImuDataCallback(ImuDataCallback, this);
    //version
    this->Version = new LivoxLidarSdkVer;
    //GetLivoxLidarSdkVer(this->Version);
    //info change
    //SetLivoxLidarInfoChangeCallback(LidarInfoChangeCallback, this);

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
  else if(request->command == 3){
    RCLCPP_INFO(get_logger(), "[Service] Reboot"); // Log comando di reboot
    LivoxLidarSdkUninit();  

    // invia il reboot
    //RCLCPP_INFO(get_logger(), "[Service] Rebooting lidar handle=%u", handlePc2);
    SetLivoxLidarInfoChangeCallback(prova, this);

    if (!LivoxLidarSdkInit(cfg_path.c_str())) {
      printf("Livox Init Failed\n");  // Stampa errore in console se fallisce
      LivoxLidarSdkUninit();         // Pulizia del driver
    }
    LivoxLidarSdkStart();

    // Inizializza il driver Livox con il file di configurazione


    response->success = true;                       // Segnala successo

  }else if(request->command == 4){

    /*
    // invia il reboot
    //RCLCPP_INFO(get_logger(), "[Service] Rebooting lidar handle=%u", handlePc2);
    SetLivoxLidarInfoChangeCallback(prova1, this);
    SetLivoxLidarInfoChangeCallback(LidarInfoChangeCallback, this);

    // Inizializza il driver Livox con il file di configurazione
    */


    response->success = true;     
  }else if(request->command == 5){
    /*
        if (!LivoxLidarSdkInit(cfg_path.c_str())) {
      printf("Livox Init Failed\n");  // Stampa errore in console se fallisce
      LivoxLidarSdkUninit();         // Pulizia del driver
    }
    SetLivoxLidarInfoChangeCallback(LidarInfoChangeCallback, this);
    */
    response->success = true;     
  }
  else {

    printf("\n\n\n******************TEST******************\n\n\n");
    RCLCPP_WARN(get_logger(), "[Service] Comando non valido: %d", request->command);
    response->success = false;                      // Segnala fallimento
  }
}

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



void Mid360_Server::on_pub_timer_INFO()
{
  //printf("\n**************     INFO SEND    *****************\n");

  livox_lidar_interfaces::msg::LivoxInfo info_msg;
  
  //auto self = static_cast<Mid360_Server*>(client_data);
  
  printf("dev_type=%d\n",this->send_info_msg.lidar_info.dev_type);
  printf("serial_number=%s\n",this->send_info_msg.lidar_info.sn);
  printf("lidar_ip_address= %s\n",this->send_info_msg.lidar_info.lidar_ip);
  printf("Point Data Type (valore numerico): %d\n", (int)this->send_info_msg.point_data_type);
  printf("Scan Pattern (valore numerico): %d\n", (int)this->send_info_msg.scan_pattern);
  printf("Frame Rate (valore numerico): %d\n", (int)this->send_info_msg.point_frame_rate);
  printf("Work Mode (valore numerico): %d\n", (int)this->send_info_msg.work_mode);
  
  info_msg.serial_number = std::string(this->send_info_msg.lidar_info.sn);
  info_msg.lidar_ip_address = std::string(this->send_info_msg.lidar_info.lidar_ip);
  info_msg.device_type = this->send_info_msg.lidar_info.dev_type;
  info_msg.point_data_type = std::to_string((int)this->send_info_msg.point_data_type);
  info_msg.scan_pattern = std::to_string((int)this->send_info_msg.scan_pattern);
  info_msg.frame_rate = (int)this->send_info_msg.point_frame_rate;
  info_msg.work_mode = std::to_string((int)this->send_info_msg.work_mode);
  
  /*
  info_msg.serial_number = this->info_livox ? 
                         std::string(this->info_livox) : 
                         "";

  info_msg.lidar_ip_address = (this->info_addr.s_addr != 0) ?
                            std::string(inet_ntoa(this->info_addr)) : "";*/
                            
  //info_msg;


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
