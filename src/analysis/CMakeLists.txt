set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

find_package(PkgConfig REQUIRED)
pkg_check_modules(PCL_IO pcl_io-1.7)
if(PCL_IO_FOUND)
  message("---- PCL IO was found (in analysis)\n")
  set(PCL_IO_PKG_CONFIG "pcl_io-1.7")
  set(USE_PRONTO_VIS_PCL TRUE)
endif()

pkg_check_modules(PCL_IO_1_8 pcl_io-1.8)
if(PCL_IO_1_8_FOUND)
  message("---- PCL IO_1_8 was found (in analysis)\n")
  set(PCL_IO_PKG_CONFIG "pcl_io-1.8")
  set(USE_PRONTO_VIS_PCL TRUE)
endif()

if(NOT PCL_IO_FOUND AND NOT PCL_IO_1_8_FOUND)
  message("---- PCL IO not found in analysis\n")
endif()

##############################################
add_executable (doMatching2D doMatching2D.cpp)
target_link_libraries(doMatching2D boost_system ${VTK_LIBRARIES})
pods_use_pkg_config_packages(doMatching2D eigen3
  lcm    bot2-frames
  lcmtypes_bot2-core  
  lidar-odom icp-registration
  ${FK_PACKAGES})
pods_install_executables(doMatching2D)

add_executable (icp-error-compute icp_error_compute.cpp)
target_link_libraries(icp-error-compute boost_system ${VTK_LIBRARIES})
pods_use_pkg_config_packages(icp-error-compute eigen3
  lcm    bot2-frames
  lcmtypes_bot2-core  
  lidar-odom ${REQUIRED_PACKAGES} ${PCL_IO_PKG_CONFIG}
  pronto_vis icp-registration
  ${FK_PACKAGES})
pods_install_executables(icp-error-compute)

add_executable (perturbations_routine perturbations_routine_app.cpp)
target_link_libraries(perturbations_routine boost_system filteringUtils drawingUtils icp-registration)
pods_use_pkg_config_packages(perturbations_routine eigen3
  lcm    bot2-frames
  lcmtypes_bot2-core  
  lidar-odom ${REQUIRED_PACKAGES} ${PCL_IO_PKG_CONFIG}
  pronto_vis icp-registration pronto_vis
  bot2-lcmgl-client 
  lidar-odom laser-util
  ${FK_PACKAGES})
pods_install_executables(perturbations_routine)



