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

void LidarInfoChangeCallback(const uint32_t handle, const LivoxLidarInfo* info, void* client_data) {
  printf("\n::::::::::::::::Entrato:::::::::::::::::\n");
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