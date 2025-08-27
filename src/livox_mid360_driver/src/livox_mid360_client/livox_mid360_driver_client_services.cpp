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