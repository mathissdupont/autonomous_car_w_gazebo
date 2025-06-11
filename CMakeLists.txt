cmake_minimum_required(VERSION 3.10)
project(ahaltech_sim)

# C++ standardı
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ROS 2 paketleri
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rcl_interfaces REQUIRED)


# İnclude dizinleri
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(sensor_cleaner src/sensor_cleaner.cpp)
ament_target_dependencies(sensor_cleaner rclcpp sensor_msgs std_msgs cv_bridge OpenCV)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)


# Yürütülebilirler
install(TARGETS
  sensor_cleaner
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/decision_maker.py
  scripts/pid_controller.py
  DESTINATION lib/${PROJECT_NAME}
)


# Paket bildirimi
ament_package()
