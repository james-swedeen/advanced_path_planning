cmake_minimum_required(VERSION 3.8)
project(rrt_search)
if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

## Normal Flags
add_compile_options(-std=c++17 -fopenmp)
add_link_options(-Wl,--no-as-needed -fno-builtin-malloc -fno-builtin-calloc -fno-builtin-realloc -fno-builtin-free -ljemalloc -Wl,--as-needed)
## Debugging Flags
#add_compile_options(-march=native -mtune=native -g3 -ggdb3 -O0 -fno-inline -fno-omit-frame-pointer -DEIGEN_USE_LAPACKE)
#add_compile_options(-march=native -mtune=native -g3 -ggdb3 -O3 -fno-inline -fno-omit-frame-pointer -DEIGEN_USE_LAPACKE)
#add_compile_options(-DNDEBUG)
#add_compile_options(-fsanitize=leak)
#add_link_options(-fsanitize=leak)
#add_compile_options(-fsanitize=undefined -fsanitize=address -fsanitize=pointer-compare -fsanitize=pointer-subtract --param=max-vartrack-size=0) # SetEnvironmentVariable('ASAN_OPTIONS', 'detect_invalid_pointer_pairs=2:new_delete_type_mismatch=0')
#add_link_options(-fsanitize=undefined -fsanitize=address -fsanitize=pointer-compare -fsanitize=pointer-subtract)
#add_compile_options(-fsanitize=thread -DEIGEN_DONT_PARALLELIZE) # SetEnvironmentVariable('TSAN_OPTIONS', 'report_atomic_races=0 suppressions=/home/james/ros_ws/thread_sanitizer_suppress.txt')
#add_link_options(-fsanitize=thread)
## Release Flags
add_compile_options(-O3 -march=native -mtune=native -DNDEBUG -fno-math-errno -DEIGEN_USE_LAPACKE -DEIGEN_STACK_ALLOCATION_LIMIT=0 -DEIGEN_UNROLLING_LIMIT=1000 -ftemplate-depth=1100)
#add_compile_options(-O2 -march=native -mtune=native -DNDEBUG -fno-math-errno -DEIGEN_USE_LAPACKE)
## Profiling Flags
#add_compile_options(-O3 -march=native -mtune=native -g3 -fno-inline -fno-omit-frame-pointer -DNDEBUG -DEIGEN_USE_LAPACKE -DEIGEN_STACK_ALLOCATION_LIMIT=0 -DEIGEN_UNROLLING_LIMIT=1000 -ftemplate-depth=1100)
#add_link_options(-Wl,--no-as-needed  -fno-builtin-malloc -fno-builtin-calloc -fno-builtin-realloc -fno-builtin-free -ljemalloc -lprofiler -Wl,--as-needed)

##################
## Dependencies ##
##################

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(kalman_filter REQUIRED)
find_package(matplotlibcpp REQUIRED)
find_package(occupancy_grid REQUIRED)
find_package(radar_detection REQUIRED)
find_package(visibility_graph REQUIRED)

find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(uav_interfaces REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(flann REQUIRED flann)
pkg_check_modules(LZ4 REQUIRED liblz4)
find_package(LAPACK REQUIRED)
find_package(TBB REQUIRED)
set(TBB_USE_TBBMALLOC OFF)

###########
## Build ##
###########

## Radar Path Planning Demo
add_executable(radar_path_planning_demo
  src/radar_path_planning_demo.cpp
)
target_include_directories(radar_path_planning_demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(radar_path_planning_demo
  rclcpp
  kalman_filter
  matplotlibcpp
  occupancy_grid
  radar_detection
  visibility_graph
  OpenMP
  Eigen3
  TBB
  LAPACK
)
target_link_libraries(radar_path_planning_demo
  OpenMP::OpenMP_CXX
  TBB::tbb
)

## Open loop error budget analysis Demo
add_executable(ol_error_budget_demo
  src/ol_error_budget_demo.cpp
)
target_include_directories(ol_error_budget_demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(ol_error_budget_demo
  rclcpp
  kalman_filter
  matplotlibcpp
  OpenMP
  Eigen3
  TBB
  LAPACK
)
target_link_libraries(ol_error_budget_demo
  OpenMP::OpenMP_CXX
  TBB::tbb
)

## Closed loop Radar Path Planning Demo
add_executable(cl_pd_path_planning_demo
  src/cl_pd_path_planning_demo.cpp
)
target_include_directories(cl_pd_path_planning_demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(cl_pd_path_planning_demo
  rclcpp
  kalman_filter
  matplotlibcpp
  radar_detection
  visibility_graph
  OpenMP
  Eigen3
  TBB
  LAPACK
)
target_link_libraries(cl_pd_path_planning_demo
  OpenMP::OpenMP_CXX
  TBB::tbb
)

## Steady State Covariance Demo
add_executable(steady_state_cov_demo
  src/steady_state_cov_demo.cpp
)
target_include_directories(steady_state_cov_demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(steady_state_cov_demo
  rclcpp
  kalman_filter
  matplotlibcpp
  occupancy_grid
  radar_detection
  OpenMP
  Eigen3
  TBB
  LAPACK
)
target_link_libraries(steady_state_cov_demo
  OpenMP::OpenMP_CXX
  TBB::tbb
)

## Gauss Markov Fitting Demo
add_executable(gauss_markov_fitting_demo
  src/gauss_markov_fitting_demo.cpp
)
target_include_directories(gauss_markov_fitting_demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(gauss_markov_fitting_demo
  kalman_filter
  OpenMP
  Eigen3
  TBB
  LAPACK
)
target_link_libraries(gauss_markov_fitting_demo
  OpenMP::OpenMP_CXX
  TBB::tbb
)

## Waypoint Path Planner
add_executable(waypoint_path_planner
  src/waypoint_path_planner_node.cpp
)
target_include_directories(waypoint_path_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(waypoint_path_planner
  rclcpp
  occupancy_grid
  geometry_msgs
  diagnostic_msgs
  std_srvs
  uav_interfaces
  OpenMP
  Eigen3
  TBB
  LAPACK
)
target_link_libraries(waypoint_path_planner
  OpenMP::OpenMP_CXX
  TBB::tbb
)

## Path Planing demo
add_executable(path_planning_demo
  src/path_planner_demo.cpp
)
target_include_directories(path_planning_demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(path_planning_demo
  rclcpp
  matplotlibcpp
  occupancy_grid
  OpenMP
  Eigen3
  TBB
  LAPACK
)
target_link_libraries(path_planning_demo
  OpenMP::OpenMP_CXX
  TBB::tbb
)

## Dubin's Path Planing demo
add_executable(dubins_path_planning_demo
  src/dubins_path_planner_demo.cpp
)
target_include_directories(dubins_path_planning_demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(dubins_path_planning_demo
  rclcpp
  matplotlibcpp
  occupancy_grid
  OpenMP
  Eigen3
  TBB
  LAPACK
)
target_link_libraries(dubins_path_planning_demo
  OpenMP::OpenMP_CXX
  TBB::tbb
)

## Fillet Path Planing demo
add_executable(fillet_path_planning_demo
  src/fillet_path_planner_demo.cpp
)
target_include_directories(fillet_path_planning_demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(fillet_path_planning_demo
  rclcpp
  matplotlibcpp
  occupancy_grid
  OpenMP
  Eigen3
  TBB
  LAPACK
)
target_link_libraries(fillet_path_planning_demo
  OpenMP::OpenMP_CXX
  TBB::tbb
)

## Reverse Fillet Path Planing demo
add_executable(reverse_fillet_path_planning_demo
  src/reverse_fillet_path_planner_demo.cpp
)
target_include_directories(reverse_fillet_path_planning_demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(reverse_fillet_path_planning_demo
  rclcpp
  matplotlibcpp
  occupancy_grid
  OpenMP
  Eigen3
  TBB
  LAPACK
)
target_link_libraries(reverse_fillet_path_planning_demo
  OpenMP::OpenMP_CXX
  TBB::tbb
)

############
## Export ##
############

ament_export_include_directories(include)
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(
  kalman_filter
  occupancy_grid
  radar_detection
  visibility_graph
  rclcpp
  nav_msgs
  std_srvs
  geometry_msgs
  diagnostic_msgs
  uav_interfaces
  rviz_visual_tools
  tf2
  tf2_eigen
  tf2_geometry_msgs
  Eigen3
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
  waypoint_path_planner
  #radar_path_planning_demo
  #ol_error_budget_demo
  cl_pd_path_planning_demo
  #steady_state_cov_demo
  gauss_markov_fitting_demo
  path_planning_demo
  dubins_path_planning_demo
  fillet_path_planning_demo
  reverse_fillet_path_planning_demo
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  set(${PROJECT_NAME}_tests
    rrt_math_test
    random_point_generator_test
    circle_point_generator_test
    ellipse_point_generator_test
    edge_generator_test
    rx200_arm_edge_generator_test
    arc_fillet_edge_generator_test
    fermat_spiral_edge_generator_test
    euler_spiral_edge_generator_test
    cubic_bezier_curve_generator_test
    nearest_neighbor_searcher_test
  )
  ament_add_gtest(rrt_math_test test/helpers/rrt_math_test.cpp)
  ament_add_gtest(random_point_generator_test test/samplers/point_generators/random_point_generator_test.cpp)
  ament_add_gtest(circle_point_generator_test test/samplers/point_generators/circle_point_generator_test.cpp)
  ament_add_gtest(ellipse_point_generator_test test/samplers/point_generators/ellipse_point_generator_test.cpp)
  ament_add_gtest(edge_generator_test test/edge_generators/edge_generator_test.cpp)
  ament_add_gtest(rx200_arm_edge_generator_test test/edge_generators/rx200_arm_edge_generator_test.cpp)
  ament_add_gtest(arc_fillet_edge_generator_test test/edge_generators/fillets/arc_fillet_edge_generator_test.cpp)
  ament_add_gtest(fermat_spiral_edge_generator_test test/edge_generators/fillets/fermat_spiral_edge_generator_test.cpp)
  ament_add_gtest(euler_spiral_edge_generator_test test/edge_generators/fillets/euler_spiral_edge_generator_test.cpp)
  ament_add_gtest(cubic_bezier_curve_generator_test test/edge_generators/fillets/cubic_bezier_curve_generator_test.cpp)
  ament_add_gtest(nearest_neighbor_searcher_test test/tree/kd_tree/nearest_neighbor_searcher_test.cpp)

  foreach(test_it ${${PROJECT_NAME}_tests})
    target_include_directories(${test_it} PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>)
    ament_target_dependencies(${test_it}
      Eigen3
      OpenMP
    )
    target_link_libraries(${test_it}
      OpenMP::OpenMP_CXX
      TBB::tbb
    )
  endforeach()
endif()

ament_package()

