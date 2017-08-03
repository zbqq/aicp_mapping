set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(aicp)

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

############## overlap from octrees library
add_library(aicpOverlap SHARED impl/octrees_overlap.cpp)
set_target_properties(aicpOverlap PROPERTIES SOVERSION 1)
target_link_libraries(aicpOverlap
  lcm    bot2-frames lcm boost_system
                       boost_thread)
pods_install_libraries(aicpOverlap)
pods_install_headers(include/common.hpp include/abstract_overlapper.hpp impl/octrees_overlap.hpp impl/overlap.hpp DESTINATION aicpOverlap)
pods_use_pkg_config_packages(aicpOverlap eigen3
  lcm    bot2-frames
  path-util
  lcmtypes_bot2-core octomap
  lidar-odom ${REQUIRED_PACKAGES} pcl_io-1.7
  octomap-util
  ${FK_PACKAGES})
pods_install_pkg_config_file(aicpOverlap
    LIBS -laicp-overlap
    REQUIRES 
    VERSION 0.0.1)

############## overlap from octrees executable
#add_executable (octrees-diff octrees_diff_app.cpp)
#target_link_libraries(octrees-diff pointmatcher lcm boost_system
#                       boost_thread octrees-difference)
#pods_use_pkg_config_packages(octrees-diff eigen3
#  lcm    bot2-frames
#  path-util
#  lcmtypes_bot2-core octomap
#  lidar-odom ${REQUIRED_PACKAGES} pcl_io-1.7
#  pronto_vis icp-registration
#  octomap-util
#  ${FK_PACKAGES})
#pods_install_executables(octrees-diff)




