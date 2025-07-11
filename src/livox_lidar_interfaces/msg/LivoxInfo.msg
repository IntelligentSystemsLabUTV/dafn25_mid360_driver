# Custom ROS2 message for Livox LiDAR status information
#
# June 11, 2025

std_msgs/Header header

# LivoxLidarSdkVer
int32 sdk_version_major
int32 sdk_version_minor
int32 sdk_version_patch

# LivoxLidarInfo
uint8 device_type_info
string serial_number
string lidar_ip_address

# LivoxLidarDeviceType
string device_type

# LivoxLidarPointDataType
string point_data_type

# LivoxLidarStatus
string lidar_status

# LivoxLidarScanPattern
string scan_pattern

# LivoxLidarPointFrameRate
uint8 frame_rate

# LivoxLidarWorkMode
string work_mode

# LivoxLidarWorkModeAfterBoot
string work_mode_after_boot

# LivoxLidarDetectMode
string detect_mode
