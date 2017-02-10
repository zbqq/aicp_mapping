set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

##############################################
add_executable (sequential-registration seq-registration-app.cpp aligned_sweeps_collection.cpp sweep_scan.cpp lidar_scan.cpp)
target_link_libraries(sequential-registration pthread boost_system boost_filesystem pointmatcher cloud_accumulate filteringUtils drawingUtils timingUtils ${VTK_LIBRARIES})
pods_use_pkg_config_packages(sequential-registration eigen3
  lcm    bot2-frames icp-registration pronto_vis
  lcmtypes_bot2-core ${REQUIRED_PACKAGES} pcl_io-1.7 bot2-lcmgl-client 
  lidar-odom laser-util
  ${FK_PACKAGES})
pods_install_executables(sequential-registration)

add_executable (error-from-gt-map seq-registration-error-from-gt-map.cpp aligned_sweeps_collection.cpp sweep_scan.cpp lidar_scan.cpp)
target_link_libraries(error-from-gt-map pthread boost_system pointmatcher cloud_accumulate filteringUtils drawingUtils timingUtils ${VTK_LIBRARIES})
pods_use_pkg_config_packages(error-from-gt-map eigen3
  lcm    bot2-frames icp-registration pronto_vis
  lcmtypes_bot2-core ${REQUIRED_PACKAGES} pcl_io-1.7 bot2-lcmgl-client
  lidar-odom laser-util
  ${FK_PACKAGES})
pods_install_executables(error-from-gt-map)
