cmake_minimum_required(VERSION 3.10)
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

set(THREADS_PREFER_PTHREAD_FLAG ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(dua_qos_cpp REQUIRED)
find_package(livox_lidar_interfaces REQUIRED)
find_package(params_manager_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Build standalone application
#! You have to rearrange this to build the node as a component later
add_executable(livox_mid360_driver
  src/livox_mid360_driver.cpp)
target_include_directories(livox_mid360_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(livox_mid360_driver PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17
target_link_libraries(livox_mid360_driver
  livox_lidar_sdk_shared)
ament_target_dependencies(livox_mid360_driver
  dua_qos_cpp
  livox_lidar_interfaces
  params_manager_cpp
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  std_srvs
)

# Install standalone application
install(TARGETS livox_mid360_driver
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
