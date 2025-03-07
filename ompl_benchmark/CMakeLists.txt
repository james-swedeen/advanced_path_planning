cmake_minimum_required(VERSION 3.8)
project(ompl_benchmark)

## Normal Flags
add_compile_options(-std=c++17 -fopenmp)
add_link_options(-Wl,--no-as-needed -fno-builtin-malloc -fno-builtin-calloc -fno-builtin-realloc -fno-builtin-free -ljemalloc -Wl,--as-needed)

## Debugging Flags
#add_compile_options(-march=native -mtune=native -g3 -ggdb3 -O0 -fno-omit-frame-pointer)
#add_compile_options(-fsanitize=address)
#add_link_options(-fsanitize=address)
## Release Flags
add_compile_options(-O3 -march=native -mtune=native -DNDEBUG -fno-math-errno -DEIGEN_USE_LAPACKE -DEIGEN_STACK_ALLOCATION_LIMIT=0 -DEIGEN_UNROLLING_LIMIT=1000 -ftemplate-depth=1100 -fno-PIC)
## Profiling Flags
#add_compile_options(-O3 -march=native -mtune=native -g3 -fno-inline -fno-omit-frame-pointer -DNDEBUG -DEIGEN_USE_LAPACKE -DEIGEN_STACK_ALLOCATION_LIMIT=0 -DEIGEN_UNROLLING_LIMIT=1000 -ftemplate-depth=1100)
#add_link_options(-Wl,--no-as-needed  -fno-builtin-malloc -fno-builtin-calloc -fno-builtin-realloc -fno-builtin-free -ljemalloc -lprofiler -Wl,--as-needed)

##################
## Dependencies ##
##################

find_package(ament_cmake REQUIRED)

find_package(rrt_search REQUIRED)
find_package(occupancy_grid REQUIRED)
find_package(kalman_filter REQUIRED)
find_package(radar_detection REQUIRED)

find_package(rclcpp REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)
find_package(OpenMP REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(flann REQUIRED flann)
pkg_check_modules(LZ4 REQUIRED liblz4)
find_package(TBB REQUIRED)
set(TBB_USE_TBBMALLOC OFF)

###########
## Build ##
###########

## Benchmark Node
add_executable(benchmark_node
  src/benchmark_node.cpp
)
target_include_directories(benchmark_node PUBLIC
  ${OMPL_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(benchmark_node
  rrt_search
  occupancy_grid
  rclcpp
  Eigen3
  ompl
  OpenMP
  TBB
)
target_link_libraries(benchmark_node
  OpenMP::OpenMP_CXX
  TBB::tbb
  ${OMPL_LIBRARIES}
)

## PD Benchmark Node
add_executable(pd_benchmark_node
  src/pd_benchmark_node.cpp
)
target_include_directories(pd_benchmark_node PUBLIC
  ${OMPL_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(pd_benchmark_node
  rrt_search
  rclcpp
  kalman_filter
  radar_detection
  OpenMP
  Eigen3
  TBB
  LAPACK
  ompl
)
target_link_libraries(pd_benchmark_node
  OpenMP::OpenMP_CXX
  TBB::tbb
  ${OMPL_LIBRARIES}
)

############
## Export ##
############

ament_export_include_directories(include)
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(
  rrt_search
  occupancy_grid
  kalman_filter
  radar_detection
  rclcpp
  Eigen3
  ompl
  OpenMP
  TBB
  LAPACK
)

#############
## Install ##
#############

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  benchmark_node
  pd_benchmark_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

#############
## Testing ##
#############


ament_package()

