cmake_minimum_required(VERSION 3.0.2)
project(social_nav_metrics)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  std_msgs
  pedsim_msgs
)

find_package(ompl REQUIRED)
find_package(Octomap REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBFCL REQUIRED fcl)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBFCL_PC REQUIRED fcl)
# find *absolute* paths to LIBFCL_* paths
find_path(LIBFCL_INCLUDE_DIRS fcl/config.h HINTS ${LIBFCL_PC_INCLUDE_DIR} ${LIBFCL_PC_INCLUDE_DIRS})
set(LIBFCL_LIBRARIES)
foreach(_lib ${LIBFCL_PC_LIBRARIES})
  find_library(_lib_${_lib} ${_lib} HINTS ${LIBFCL_PC_LIBRARY_DIRS})
  list(APPEND LIBFCL_LIBRARIES ${_lib_${_lib}})
endforeach()

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs nav_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  ${FCL_INCLUDE_DIRS}
)

add_executable(collision_counter src/collision_counter.cpp)
target_link_libraries(collision_counter ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${LIBFCL_LIBRARIES} ccd ${OCTOMAP_LIBRARIES} ${OMPL_LIBRARIES})
add_dependencies(collision_counter ${catkin_EXPORTED_TARGETS})
