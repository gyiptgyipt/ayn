cmake_minimum_required(VERSION 3.8)
project(point_cloud_merger)

# Set CMake policy to avoid PCL_ROOT warnings
cmake_policy(SET CMP0074 NEW)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Modern PCL finding approach for ROS 2
find_package(PCL REQUIRED COMPONENTS common io)
find_package(pcl_conversions REQUIRED)

# Include directories
include_directories(
  ${PCL_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
)

# Add executable
add_executable(point_cloud_merger src/point_cloud_merger.cpp)

# Target linking - using both modern and legacy approaches
target_link_libraries(point_cloud_merger
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${message_filters_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  ${pcl_conversions_LIBRARIES}
  ${PCL_LIBRARIES}
)

# Only include ROS 2 packages in ament_target_dependencies
ament_target_dependencies(point_cloud_merger
  rclcpp
  sensor_msgs
  message_filters
  tf2_ros
  pcl_conversions
  tf2_sensor_msgs
  visualization_msgs
)

# Install targets
install(TARGETS
  point_cloud_merger
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/merge_pointclouds.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
  DESTINATION
    share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()