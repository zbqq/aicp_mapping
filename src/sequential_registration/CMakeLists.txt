set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

find_package(PkgConfig REQUIRED)
pkg_check_modules(PCL_IO pcl_io-1.7)
if(PCL_IO_FOUND)
  message("---- PCL IO was found (in sequential_registration)\n")
  set(PCL_IO_PKG_CONFIG "pcl_io-1.7")
  set(USE_PRONTO_VIS_PCL TRUE)
endif()

pkg_check_modules(PCL_IO_1_8 pcl_io-1.8)
if(PCL_IO_1_8_FOUND)
  message("---- PCL IO_1_8 was found (in sequential_registration)\n")
  set(PCL_IO_PKG_CONFIG "pcl_io-1.8")
  set(USE_PRONTO_VIS_PCL TRUE)
endif()

if(NOT PCL_IO_FOUND AND NOT PCL_IO_1_8_FOUND)
  message("---- PCL IO not found in sequential_registration\n")
endif()

##############################################
add_executable (sequential-registration seq-registration-app.cpp aligned_sweeps_collection.cpp sweep_scan.cpp lidar_scan.cpp)
target_link_libraries(sequential-registration pthread boost_system boost_filesystem pointmatcher cloud_accumulate filteringUtils drawingUtils timingUtils ${VTK_LIBRARIES})
pods_use_pkg_config_packages(sequential-registration eigen3
  lcm    bot2-frames icp-registration pronto_vis
  lcmtypes_bot2-core ${REQUIRED_PACKAGES} ${PCL_IO_PKG_CONFIG} bot2-lcmgl-client 
  lidar-odom laser-util
  ${FK_PACKAGES})
pods_install_executables(sequential-registration)

add_executable (error-from-gt-map seq-registration-error-from-gt-map.cpp aligned_sweeps_collection.cpp sweep_scan.cpp lidar_scan.cpp)
target_link_libraries(error-from-gt-map pthread boost_system pointmatcher cloud_accumulate filteringUtils drawingUtils timingUtils ${VTK_LIBRARIES})
pods_use_pkg_config_packages(error-from-gt-map eigen3
  lcm    bot2-frames icp-registration pronto_vis
  lcmtypes_bot2-core ${REQUIRED_PACKAGES} ${PCL_IO_PKG_CONFIG} bot2-lcmgl-client
  lidar-odom laser-util
  ${FK_PACKAGES})
pods_install_executables(error-from-gt-map)
