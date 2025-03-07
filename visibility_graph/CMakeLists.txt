cmake_minimum_required(VERSION 3.8)
project(visibility_graph)

##################
## Dependencies ##
##################

find_package(ament_cmake REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(TBB REQUIRED)

###########
## Build ##
###########

############
## Export ##
############

ament_export_include_directories(include)
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(
  Eigen3
  OpenMP
  TBB
)

#############
## Install ##
#############

install(
  DIRECTORY include/
  DESTINATION include
)

#############
## Testing ##
#############

ament_package()

