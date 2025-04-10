cmake_minimum_required(VERSION 3.5)
project(super_odometry)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++1z")
set(CMAKE_CXX_FLAGS_RELEASE "-O4 -Wall -g")
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
SET(TBB_INCLUDE_DIR "/usr/include/tbb")
add_compile_options(-Wall -Wextra -Wpedantic -Wunused-parameter)



# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra )
endif()

if (POLICY CMP0048)
  cmake_policy(SET CMP0048 NEW)
endif (POLICY CMP0048)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(super_odometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Ceres REQUIRED)
find_package(GTSAM REQUIRED QUIET)
find_package(TBB REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${TBB_INCLUDE_DIR}
  ${Sophus_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
  ${livox_ros_driver2_INCLUDE_DIRS}
)

# Link directories
link_directories(
  include
  ${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${GTSAM_LIBRARY_DIRS}
)

set(GTSAM_LIBRARIES gtsam)


# SuperOdom Library
add_library(SuperOdomLib SHARED
  src/FeatureExtraction/featureExtraction.cpp
  src/ImuPreintegration/imuPreintegration.cpp
  src/LaserMapping/laserMapping.cpp
  src/LaserMapping/lidarOptimization.cpp
  src/LaserMapping/LocalMap.cpp
  src/LidarProcess/LidarSlam.cpp
  src/LidarProcess/pose_local_parameterization.cpp
  src/LidarProcess/SE3AbsolutatePoseFactor.cpp
  src/parameter/parameter.cpp
  src/utils/superodom_utils.cpp
)

# Set up library include directory
target_include_directories(SuperOdomLib PUBLIC  
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/SuperOdomLib>  
    $<INSTALL_INTERFACE:include/SuperOdomLib>  # <prefix>/include/mylib
)

ament_target_dependencies(SuperOdomLib
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs 
  sensor_msgs
  image_transport
  cv_bridge
  tf2
  tf2_ros
  tf2_geometry_msgs
  super_odometry_msgs
  visualization_msgs
  pcl_conversions
  livox_ros_driver2
)
target_link_libraries(SuperOdomLib
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  gtsam
 ${CERES_LIBRARIES}
)

# IMU PreIntegration node
add_executable(imu_preintegration_node src/imuPreintegration_node.cpp)
ament_target_dependencies(imu_preintegration_node rclcpp nav_msgs sensor_msgs tf2_ros)
target_link_libraries(imu_preintegration_node
    gtsam
    SuperOdomLib
)



# Feature Extraction node
add_executable(feature_extraction_node src/featureExtraction_node.cpp)
ament_target_dependencies(feature_extraction_node rclcpp nav_msgs sensor_msgs super_odometry_msgs livox_ros_driver2)
target_link_libraries(feature_extraction_node 
  ${PCL_LIBRARIES}
  ${TBB_LIBRARIES}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  SuperOdomLib
)

# LiDAR Mapping node
add_executable(laser_mapping_node src/laserMapping_node.cpp)
ament_target_dependencies(laser_mapping_node rclcpp std_msgs nav_msgs sensor_msgs geometry_msgs tf2 tf2_ros)
target_link_libraries(laser_mapping_node
    ${PCL_LIBRARIES}
    ${CERES_LIBRARIES}
    SuperOdomLib
)

# Install Exported Libraries
ament_export_targets(export_super_odometry HAS_LIBRARY_TARGET)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS SuperOdomLib
  EXPORT export_super_odometry
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(SuperOdomLib)

#Install executables
install(TARGETS
  imu_preintegration_node
  feature_extraction_node
  laser_mapping_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
        DESTINATION share/${PROJECT_NAME}/
)

ament_package()
