
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

    } else if (cmd == 0)  {
        std::cout << "Hai scelto DISABLE.\n";

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
    }

  

  //! Note: this time we don't spin, we just call a method offered by the node
  client_node->Enable_Disable_srv(cmd);

  // Just exit
  rclcpp::shutdown();
  exit(EXIT_SUCCESS);
}
