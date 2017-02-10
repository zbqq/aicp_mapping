set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

##############
add_library(octrees-difference SHARED convert_octomap.cpp)
pods_use_pkg_config_packages(octrees-difference eigen3
  lcm    bot2-frames
  path-util
  lcmtypes_bot2-core octomap
  lidar-odom ${REQUIRED_PACKAGES} pcl_io-1.7
  octomap-util
  ${FK_PACKAGES})

target_link_libraries(octrees-difference
  lcm    bot2-frames lcm boost_system
                       boost_thread)
set_target_properties(octrees-difference PROPERTIES SOVERSION 1)
pods_install_libraries(octrees-difference)
pods_install_headers(convert_octomap.hpp DESTINATION octrees-difference)

pods_install_pkg_config_file(octrees-difference
    LIBS -loctrees-difference
    REQUIRES 
    VERSION 0.0.1)

add_executable (octrees-diff octrees_diff_app.cpp)
target_link_libraries(octrees-diff pointmatcher lcm boost_system
                       boost_thread octrees-difference)
pods_use_pkg_config_packages(octrees-diff eigen3
  lcm    bot2-frames
  path-util
  lcmtypes_bot2-core octomap
  lidar-odom ${REQUIRED_PACKAGES} pcl_io-1.7
  pronto_vis icp-registration
  octomap-util
  ${FK_PACKAGES})
pods_install_executables(octrees-diff)




