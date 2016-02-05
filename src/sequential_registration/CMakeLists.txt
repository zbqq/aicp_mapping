set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(sequential_registration)

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

##############################################
add_executable (sequential-registration seq-registration-app.cpp)
target_link_libraries(sequential-registration boost_system)
pods_use_pkg_config_packages(sequential-registration eigen3
  lcm    bot2-frames
  lcmtypes_bot2-core pcl_io-1.7
  lidar-odom icp-registration
  ${FK_PACKAGES})
pods_install_executables(sequential-registration)


##############################################
# Basic Test Program
add_executable (sequential-registration-simple seq-registration-app-simple.cpp)
target_link_libraries(sequential-registration-simple boost_system)
pods_use_pkg_config_packages(sequential-registration-simple eigen3
  lcm
  lcmtypes_bot2-core pcl_io-1.7
  lidar-odom icp-registration
  ${FK_PACKAGES})
pods_install_executables(sequential-registration-simple)



