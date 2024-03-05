cmake_minimum_required(VERSION 3.8)
project(virtual_laser_scan)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(PCL REQUIRED)
include_directories(include ${PCL_INCLUDE_DIRS})

add_executable(virtual_point_cloud src/virtual_point_cloud.cpp)
ament_target_dependencies(virtual_point_cloud
  rclcpp
  std_msgs
  sensor_msgs
  tf2_ros
  pcl_conversions
)

add_executable(virtual_laser_scan src/virtual_laser_scan.cpp)
ament_target_dependencies(virtual_laser_scan
  rclcpp
  std_msgs
  sensor_msgs
)

install(
  TARGETS virtual_point_cloud virtual_laser_scan
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
