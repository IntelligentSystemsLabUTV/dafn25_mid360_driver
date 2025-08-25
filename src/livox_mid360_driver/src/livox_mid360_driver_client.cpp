

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

Mid360_Client::Mid360_Client() : Node("client")
{
  // --- Client del servizio ---
  client_ = this->create_client<livox_lidar_interfaces::srv::MsgEnableDisable>("/msg/Mid360_Server");
  RCLCPP_INFO(this->get_logger(), "Client initialized");

  // --- Subscription PointCloud2 ---
  pc2_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/msg_MID360/PointCloud2", 10,
    std::bind(&Mid360_Client::pc2Callback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscription PointCloud2 avviata");

  // --- Subscription IMU ---
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/msg_MID360/IMU", 10,
    std::bind(&Mid360_Client::imuCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscription IMU avviata");

  // --- Subscription LivoxInfo ---
  info_sub_ = this->create_subscription<livox_lidar_interfaces::msg::LivoxInfo>(
    "/msg_MID360/INFO",
    rclcpp::QoS(10),
    std::bind(&Mid360_Client::infoCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscription LivoxInfo avviata");
}

void Mid360_Client::Enable_Disable_srv(int cmd)
{
  // Attesa disponibilità servizio
  while (!client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      throw std::runtime_error("Middleware crashed while waiting for service");
    }
    RCLCPP_WARN(this->get_logger(), "Service not available. Attendo...");
  }

  // Creazione richiesta
  auto request = std::make_shared<livox_lidar_interfaces::srv::MsgEnableDisable::Request>();
  request->set__command(cmd);

  // Invio richiesta e attesa risposta
  auto response = client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(this->shared_from_this(), response) ==
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(this->get_logger(), "Risultato chiamata servizio: %d", response.get()->success);
  } else {
    client_->remove_pending_request(response);
    RCLCPP_ERROR(this->get_logger(), "Chiamata al servizio fallita");
  }
}

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

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto client_node = std::make_shared<Mid360_Client>();
  int cmd = -1;
    std::cout << "Inserisci 1 (enable) o 0 (disable) o 2(lettura dati PC2): ";
    if (!(std::cin >> cmd) || (cmd != 0 && cmd != 1 && cmd != 2 && cmd != 3 && cmd != 4 && cmd != 5)) {
        std::cerr << "Input non valido. Devi inserire solo 0 e 1 e 2 e 3.\n";
    }

    if (cmd == 1) {
        std::cout << "Hai scelto ENABLE.\n";
        // qui chiami LivoxLidarSdkInitEnable()…
    } else if (cmd == 0)  {
        std::cout << "Hai scelto DISABLE.\n";
        // qui chiami LivoxLidarSdkInitDisable()…
    } else if (cmd == 2)  {

        std::string cmdline =
          "gnome-terminal -- bash -c '"
          "source install/local_setup.bash &&"
          "ros2 run livox_mid360_driver client; "
          "exec bash'";
        system(cmdline.c_str());
        std::cout << "Hai scelto lettura dati PC2.\n";
        auto sub_node = std::make_shared<Mid360_Client>();
        rclcpp::spin(sub_node);
    }else if (cmd == 3){
      
    
    }else if (cmd == 4){
      
    }else if (cmd == 5){}

  

  //! Note: this time we don't spin, we just call a method offered by the node
  client_node->Enable_Disable_srv(cmd);

  // Just exit
  rclcpp::shutdown();
  exit(EXIT_SUCCESS);
}
