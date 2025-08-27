message(STATUS ">>> Loading CMakeLists from: ${CMAKE_CURRENT_LIST_FILE}")

cmake_minimum_required(VERSION 3.8)
project(livox_mid360_driver)

set(CMAKE_BUILD_TYPE "RelWithDebInfo")

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


add_subdirectory(
  ${CMAKE_CURRENT_LIST_DIR}/../Livox-SDK2   # path al CMakeLists.txt del SDK2
  sdk2_build                                # cartella di build dedicata
)


# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(livox_lidar_interfaces REQUIRED)
#aggiungo
find_package(ament_index_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)       


# Server
add_executable(server
  src/livox_mid360_server/livox_mid360_driver_server_costruttore.cpp
  src/livox_mid360_driver_server_main.cpp
  src/livox_mid360_server/livox_mid360_driver_server_services.cpp
  src/livox_mid360_server/livox_mid360_driver_server_PC2_IMU.cpp
  src/livox_mid360_timer/livox_mid360_driver_timer.cpp

)

target_include_directories(server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_LIST_DIR}/../Livox-SDK2/include
  )
ament_target_dependencies(
  server
  rclcpp
  livox_lidar_interfaces
  ament_index_cpp 
  sensor_msgs
  )

target_link_libraries(server
    livox_lidar_sdk_static
    ament_index_cpp::ament_index_cpp 
)

# Client
add_executable(client 
  src/livox_mid360_client/livox_mid360_driver_client_PC2_IMU.cpp
  src/livox_mid360_client/livox_mid360_driver_client_services.cpp
  src/livox_mid360_driver_client_main.cpp
  src/livox_mid360_client/livox_mid360_driver_client_costruttore.cpp
  src/livox_mid360_timer/livox_mid360_driver_timer.cpp
)
target_include_directories(client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_LIST_DIR}/../Livox-SDK2/include)

ament_target_dependencies(
  client
  rclcpp
  livox_lidar_interfaces
  ament_index_cpp
  sensor_msgs) 

target_link_libraries(client
    livox_lidar_sdk_static
    ament_index_cpp::ament_index_cpp     
)
# Install executables
install(TARGETS server
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS client
  DESTINATION lib/${PROJECT_NAME})
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
