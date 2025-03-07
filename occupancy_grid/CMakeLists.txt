cmake_minimum_required(VERSION 3.8)
project(occupancy_grid)
if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

add_compile_options(-std=c++17 -fopenmp)
add_link_options(-Wl,--no-as-needed -fno-builtin-malloc -fno-builtin-calloc -fno-builtin-realloc -fno-builtin-free -ljemalloc -Wl,--as-needed)
#add_compile_options(-march=native -g3 -ggdb3 -O0 -fno-inline -fno-omit-frame-pointer)
add_compile_options(-O3 -march=native -DNDEBUG)

##################
## Dependencies ##
##################

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(OpenCV REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetOccupancyGrid.srv"
  DEPENDENCIES nav_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

###########
## Build ##
###########

## Occupancy Grid Class
add_library(${PROJECT_NAME}_lib STATIC
  src/occupancy_grid.cpp
  src/occupancy_grid_buildings.cpp
)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  tf2
  tf2_geometry_msgs
  tf2_ros
  sensor_msgs
  OpenMP
  OpenCV
  Eigen3
)
target_link_libraries(${PROJECT_NAME}_lib ${cpp_typesupport_target})

## Building Occupancy Grid Tester
add_executable(building_occupancy_grid_node
  src/buildings_test.cpp
)
target_include_directories(building_occupancy_grid_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(building_occupancy_grid_node
  rclcpp
  nav_msgs
  std_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  Eigen3
  OpenMP
  OpenCV
)
target_link_libraries(building_occupancy_grid_node
  ${PROJECT_NAME}_lib
  OpenMP::OpenMP_CXX
)

############
## Export ##
############

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(
  rclcpp
  rclpy
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  Eigen3
  OpenMP
  OpenCV
  rosidl_default_runtime
)

#############
## Install ##
#############

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}_lib
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS
  building_occupancy_grid_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/data_world_plotter_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

