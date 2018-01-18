cmake_minimum_required(VERSION 2.6.0)
project(aicp)

# pull in the pods macros. See cmake/pods.cmake for documentation
#set(POD_NAME aicp)
#include(cmake/pods.cmake)

# get path to aicp base
add_definitions(-DPROJECTDIR=\"${CMAKE_CURRENT_SOURCE_DIR}\")

find_package(catkin REQUIRED COMPONENTS
  bot_lcmgl octomap_utils octomap_lcmtypes pronto_vis concise_args cloud_accumulate cv_bridge
  pointmatcher_ros pcl_ros
)

#find_package(libpointmatcher CONFIG)

pkg_check_modules(yaml_cpp REQUIRED yaml-cpp)
find_package(OpenCV 3.0 QUIET)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES aicp_registration
  CATKIN_DEPENDS bot_lcmgl octomap_utils octomap_lcmtypes pronto_vis concise_args cloud_accumulate pointmatcher_ros pcl_ros
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(src/utils)
add_subdirectory(src/registration)
add_subdirectory(src/overlap)
add_subdirectory(src/classification)
add_subdirectory(src/registration_apps)


add_subdirectory(src/pcl)
add_subdirectory(src/test)

#pods_install_python_script(convert_robot_behavior python/convert_robot_behavior_type.py)
