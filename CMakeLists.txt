cmake_minimum_required(VERSION 3.8)
project(social_nav_metrics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pedsim_msgs REQUIRED)
find_package(OCTOMAP REQUIRED)
find_package(octomap_msgs REQUIRED)


include_directories(
include
SYSTEM
)

# Add links to PCL definitons and directories
include_directories(${FCL_INCLUDE_DIRS})


add_executable(collision_counter src/collision_counter.cpp)
ament_target_dependencies(collision_counter
  rclcpp
  tf2
  tf2_ros
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/metrics_recorder.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

install(TARGETS
  collision_counter
  DESTINATION lib/${PROJECT_NAME})


ament_package()