cmake_minimum_required(VERSION 3.8)
project(radar_detection)

## Normal Flags
add_compile_options(-std=c++17 -fopenmp)
add_link_options(-Wl,--no-as-needed -fno-builtin-malloc -fno-builtin-calloc -fno-builtin-realloc -fno-builtin-free -ljemalloc -Wl,--as-needed)
## Debugging Flags
#add_compile_options(-g -Og) # -fno-omit-frame-pointer -fno-inline)
## Release Flags
add_compile_options(-O3 -march=native -DNDEBUG -fomit-frame-pointer)

##################
## Dependencies ##
##################

find_package(ament_cmake REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)

###########
## Build ##
###########

# Generate Timing Node
add_executable(time_test_node
  src/time_test.cpp
)
target_include_directories(time_test_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(time_test_node
  OpenMP
  Eigen3
)

############
## Export ##
############

ament_export_include_directories(include)
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(
  OpenMP
  Eigen3
)

#############
## Install ##
#############

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  time_test_node
  DESTINATION lib/${PROJECT_NAME}
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  set(${PROJECT_NAME}_tests
    partial_derivatives_test
    radar_model_test
    constant_cross_section_model_test
    ellipsoid_cross_section_model_test
    spikeball_cross_section_model_test
  )
  ament_add_gtest(partial_derivatives_test test/partial_derivatives_test.cpp)
  ament_add_gtest(radar_model_test test/radar_model_test.cpp)
  ament_add_gtest(constant_cross_section_model_test test/cross_sections/constant_cross_section_model_test.cpp)
  ament_add_gtest(ellipsoid_cross_section_model_test test/cross_sections/ellipsoid_cross_section_model_test.cpp)
  ament_add_gtest(spikeball_cross_section_model_test test/cross_sections/spikeball_cross_section_model_test.cpp)

  foreach(test_it ${${PROJECT_NAME}_tests})
    target_include_directories(${test_it} PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>)
    ament_target_dependencies(${test_it}
      Eigen3
      OpenMP
    )
  endforeach()
endif()

ament_package()

