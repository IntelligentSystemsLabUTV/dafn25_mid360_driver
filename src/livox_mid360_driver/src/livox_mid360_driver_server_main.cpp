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




int main(int argc, char ** argv)
{
  std::cout << ">>> SERVER MAIN() STARTED <<<\n";
  rclcpp::init(argc, argv);                     // Inizializza il sistema ROS2
  auto server_node = std::make_shared<Mid360_Server>(); // Crea il nodo server
  rclcpp::spin(server_node);                     // Mantiene vivo il nodo per gestire le richieste
  rclcpp::shutdown();                            // Chiude il sistema ROS2
  exit(EXIT_SUCCESS);                            // Esce con codice 0 (successo)
}
