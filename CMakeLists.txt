cmake_minimum_required(VERSION 3.8)
project(social_nav_metrics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBFCL REQUIRED "fcl>=0.7.0")
# replace LIBFCL_LIBRARIES with full paths to the libraries
set(LIBFCL_LIBRARIES_FULL "")
foreach(LIBFCL_LIBRARY ${LIBFCL_LIBRARIES})
  find_library(${LIBFCL_LIBRARY}_LIB ${LIBFCL_LIBRARY} ${LIBFCL_LIBRARY_DIRS})
  list(APPEND LIBFCL_LIBRARIES_FULL ${${LIBFCL_LIBRARY}_LIB})
endforeach()
set(LIBFCL_LIBRARIES "${LIBFCL_LIBRARIES_FULL}")

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pedsim_msgs REQUIRED)
find_package(OCTOMAP REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)

set(dependencies
rclcpp
octomap_msgs
pedsim_msgs
std_msgs
OCTOMAP
LIBFCL)


include_directories(
include
SYSTEM
)

# Add links to PCL definitons and directories
include_directories(${FCL_INCLUDE_DIRS})
include_directories(${CCD_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
add_definitions(${EIGEN3_DEFINITIONS})

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/metrics_recorder.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})


add_executable(collision_counter src/collision_counter.cpp)
ament_target_dependencies(collision_counter
  rclcpp
  tf2
  tf2_ros
  nav_msgs
  pedsim_msgs
  octomap_msgs
  ${fcl_LIBRARIES}
  ${CCD_LIBRARIES}
)
target_link_libraries(collision_counter ${LIBFCL_LIBRARIES} ${OCTOMAP_LIBRARIES} ${OMPL_LIBRARIES})

install(TARGETS
  collision_counter
  DESTINATION lib/${PROJECT_NAME})


ament_package()