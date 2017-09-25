set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

find_package(PkgConfig REQUIRED)
pkg_check_modules(PCL_IO pcl_io-1.7)
if(PCL_IO_FOUND)
  message("---- PCL IO was found (in octrees_difference)\n")
  set(PCL_IO_PKG_CONFIG "pcl_io-1.7")
  set(USE_PRONTO_VIS_PCL TRUE)
endif()

pkg_check_modules(PCL_IO_1_8 pcl_io-1.8)
if(PCL_IO_1_8_FOUND)
  message("---- PCL IO_1_8 was found (in octrees_difference)\n")
  set(PCL_IO_PKG_CONFIG "pcl_io-1.8")
  set(USE_PRONTO_VIS_PCL TRUE)
endif()

if(NOT PCL_IO_FOUND AND NOT PCL_IO_1_8_FOUND)
  message("---- PCL IO not found in octrees_difference\n")
endif()

##############
add_library(octrees-difference SHARED convert_octomap.cpp)
pods_use_pkg_config_packages(octrees-difference eigen3
  lcm    bot2-frames
  path-util
  lcmtypes_bot2-core octomap
  lidar-odom ${REQUIRED_PACKAGES} ${PCL_IO_PKG_CONFIG}
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
target_link_libraries(octrees-diff pointmatcher lcm boost_system ${VTK_LIBRARIES}
                       boost_thread octrees-difference)
pods_use_pkg_config_packages(octrees-diff eigen3
  lcm    bot2-frames
  path-util
  lcmtypes_bot2-core octomap
  lidar-odom ${REQUIRED_PACKAGES} ${PCL_IO_PKG_CONFIG}
  pronto_vis icp-registration
  octomap-util
  ${FK_PACKAGES})
pods_install_executables(octrees-diff)




