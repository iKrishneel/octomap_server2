cmake_minimum_required(VERSION 3.5)
project(octomap_server2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(PCL 1.10 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(OpenMP REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(message_filters REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

link_directories(${PCL_LIBRARY_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(include)

add_library(octomap_server2 SHARED
  src/octomap_server.cpp
  src/transforms.cpp
  src/conversions.cpp
  )

ament_target_dependencies(octomap_server2
  rclcpp
  PCL
  pcl_conversions
  sensor_msgs
  std_msgs
  nav_msgs
  visualization_msgs
  geometry_msgs
  std_srvs
  octomap
  octomap_msgs
  message_filters
  tf2_ros
  tf2_msgs
  tf2
  tf2_geometry_msgs
  rclcpp_components
  )

target_link_libraries(octomap_server2 ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})

rclcpp_components_register_node(octomap_server2
  PLUGIN "octomap_server::OctomapServer"
  EXECUTABLE octomap_server)

install(TARGETS
  octomap_server2
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
  )

ament_package()
