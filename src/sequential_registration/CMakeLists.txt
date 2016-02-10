set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(sequential_registration)

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

##############################################
add_executable (sequential-registration seq-registration-app.cpp aligned_sweeps_collection.cpp sweep_scan.cpp lidar_scan.cpp)
target_link_libraries(sequential-registration boost_system pointmatcher ${VTK_LIBRARIES})
pods_use_pkg_config_packages(sequential-registration eigen3
  lcm    bot2-frames icp-registration drc-lcmtypes
  lcmtypes_bot2-core ${REQUIRED_PACKAGES} pcl_io-1.7
  lidar-odom maps
  ${FK_PACKAGES})
pods_install_executables(sequential-registration)


set(TARGET_NAME data-blob-test)
set(REQUIRED_PACKAGES maps)
add_executable(${TARGET_NAME} ${TARGET_NAME}.cpp)
pods_install_executables(${TARGET_NAME})
pods_use_pkg_config_packages(${TARGET_NAME} ${REQUIRED_PACKAGES})
