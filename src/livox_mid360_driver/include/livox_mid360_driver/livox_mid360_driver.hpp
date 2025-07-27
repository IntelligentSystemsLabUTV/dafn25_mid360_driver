
#ifndef LIVOX_MID360_HPP
#define LIVOX_MID360_HPP

// Includiamo le API di base di ROS2
#include <rclcpp/rclcpp.hpp>
// Includiamo il tipo di servizio MsgEnableDisable generato da livox_lidar_interfaces
#include <livox_lidar_interfaces/srv/msg_enable_disable.hpp>
#include <livox_lidar_interfaces/msg/livox_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <livox_mid360_driver/livox_lidar_api.h>
#include <livox_mid360_driver/livox_lidar_def.h>
#include <sensor_msgs/msg/point_field.hpp>
#include <arpa/inet.h>
#include <vector>

// Alias per comodità: evita di ripetere il namespace completo ogni volta
using MsgEnableDisable = livox_lidar_interfaces::srv::MsgEnableDisable;

/**
 * @brief Nodo server che espone il servizio /msg/Mid360_Server
 *        per abilitare o disabilitare il dispositivo Mid360.
 */
class Mid360_Server : public rclcpp::Node
{
public:
  /**
   * @brief Costruttore: inizializza il nodo e crea il servizio.
   */
  Mid360_Server();
  uint32_t handlePc2{};

  LivoxLidarCartesianHighRawPoint*  last_point{};
  std::vector<float> gyro{0,0,0};
  std::vector<float> acc{0,0,0};
  uint32_t num_punti{0};

  struct in_addr info_addr{};
  const char* info_livox{};
  LivoxLidarSdkVer* Version{};
  //PointDataType 
  LivoxLidarPointDataType pointDataType;
  livox_status statusDataType;
  std::vector<LivoxLidarCartesianHighRawPoint> frame_buffer_;
  uint8_t current_frame_cnt_{255}; // valore iniziale "impossibile"
  bool frame_ready_{false};

private:


  /// Handle al servizio ROS2 (shared pointer)
  rclcpp::Service<MsgEnableDisable>::SharedPtr server_;

  /**
   * @brief Callback eseguita quando arriva una richiesta di servizio.
   * @param request  Puntatore alla richiesta, contiene il comando (0/1).
   * @param response Puntatore alla risposta da popolare con il risultato.
   * @return true se la chiamata è andata a buon fine, false altrimenti.
   */
  void Mid360_Enable_Disable_clbk(
    const MsgEnableDisable::Request::SharedPtr request,
    const MsgEnableDisable::Response::SharedPtr response);

  // --- publisher ROS2 ---
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc2_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr        imu_pub_;
  rclcpp::Publisher<livox_lidar_interfaces::msg::LivoxInfo>::SharedPtr info_pub_;

  // --- timer per publish (se serve) ---
  rclcpp::TimerBase::SharedPtr pub_timer_;
  rclcpp::TimerBase::SharedPtr pub_timer_IMU_;
  rclcpp::TimerBase::SharedPtr pub_timer_INFO_;

  void on_pub_timer();
  void on_pub_timer_IMU();
  void on_pub_timer_INFO();
  
};

/**
 * @brief Nodo client che invia comandi di abilitazione/disabilitazione
 *        al servizio esposto da Mid360_Server.
 */
class Mid360_Client : public rclcpp::Node
{
public:
  /**
   * @brief Costruttore: inizializza il nodo client.
   */
  Mid360_Client();

  /**
   * @brief Invia una richiesta di enable (cmd=1) o disable (cmd=0).
   * @param cmd  Intero che indica il comando da inviare.
   */
  void Enable_Disable_srv(int cmd);

private:
    // subscriptions
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc2_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr           imu_sub_;
  rclcpp::Subscription<livox_lidar_interfaces::msg::LivoxInfo>::SharedPtr info_sub_;
  /// Handle al client ROS2 (shared pointer)
  rclcpp::Client<MsgEnableDisable>::SharedPtr client_;

    // callback
  void pc2Callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void infoCallback(const livox_lidar_interfaces::msg::LivoxInfo::SharedPtr msg);

};

#endif  // LIVOX_MID360_HPP
