cmake_minimum_required(VERSION 3.5)
project(super_odometry_msgs)

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

# find_package(catkin REQUIRED COMPONENTS
#         nav_msgs
#         std_msgs
#         message_generation
#         message_runtime
#         geometry_msgs
#         sensor_msgs
#         )
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


# add_message_files(
#         FILES
#         IterationStats.msg
#         OptimizationStats.msg
#         LaserFeature.msg
# )

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/IterationStats.msg"
  "msg/OptimizationStats.msg"
  "msg/LaserFeature.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs nav_msgs sensor_msgs
  ADD_LINTER_TESTS
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
