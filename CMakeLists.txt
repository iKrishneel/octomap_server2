cmake_minimum_required(VERSION 3.5)
project(octomap_server2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_definitions("-Wall")
add_definitions("-Wextra")
add_definitions("-Wpedantic")
add_definitions("-g")
add_definitions("-O3") # because normal release is just -O2

# remove debug info from octomap
add_definitions("-DOCTOMAP_NODEBUGOUT")
# remove warning info from PCL
set(PCL_FIND_QUIETLY INTERNAL)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(rclcpp_components REQUIRED)
find_package(PCL REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(message_filters REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_ros REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(laser_geometry REQUIRED)

find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${EIGEN3_LIBRARIES})

# include include
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  )


## --------------------------------------------------------------
## |                       Octomap server                       |
## --------------------------------------------------------------

add_library(octomap_server2 SHARED
  src/octomap_server.cpp
  )

ament_target_dependencies(octomap_server2
  rclcpp
  rclcpp_components
  PCL
  pcl_ros
  pcl_conversions
  sensor_msgs
  std_msgs
  nav_msgs
  visualization_msgs
  geometry_msgs
  std_srvs
  octomap
  octomap_ros
  octomap_msgs
  message_filters
  tf2_ros
  tf2_msgs
  tf2
  laser_geometry
  )

target_link_libraries(octomap_server2 ${EIGEN3_LIBRARIES} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})

rclcpp_components_register_node(octomap_server2
  PLUGIN "octomap_server::OctomapServer"
  EXECUTABLE octomap_server)

## --------------------------------------------------------------
## |                           install                          |
## --------------------------------------------------------------

# executables
install(
  TARGETS
  octomap_server
  DESTINATION lib/${PROJECT_NAME}
  )

# libraries
ament_export_include_directories(include)
install(
  TARGETS
  octomap_server2
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
  )

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
  )

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
  )

install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}/
  )

ament_package()
