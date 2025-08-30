cmake_minimum_required(VERSION 3.10)
project(sensor_flip)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(flip_node src/flip_node.cpp)
ament_target_dependencies(flip_node rclcpp sensor_msgs tf2 tf2_ros tf2_eigen tf2_geometry_msgs livox_ros_driver2)

# Add compilation optimizations for performance
target_compile_options(flip_node PRIVATE
  -O3 -march=native -mtune=native -funroll-loops -ffast-math
  $<$<CONFIG:Release>:-DNDEBUG>
)

# Enable parallel execution support
find_package(TBB QUIET)
if(TBB_FOUND)
  target_link_libraries(flip_node TBB::tbb)
  target_compile_definitions(flip_node PRIVATE HAVE_TBB)
endif()

target_include_directories(flip_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS flip_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
