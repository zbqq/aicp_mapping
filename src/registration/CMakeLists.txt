set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(aicp)

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

### pcl
find_package(PkgConfig REQUIRED)

pkg_check_modules(PCL_LIBS pcl_io-1.7)
if(PCL_LIBS_FOUND)
  find_package(PCL 1.7 REQUIRED)
  set(PCL_LIBRARIES "pcl_io-1.7 pcl_filters-1.7 pcl_segmentation-1.7 pcl_common-1.7 pcl_visualization-1.7")
endif()

pkg_check_modules(PCL_LIBS_1_8 pcl_io-1.8)
if(PCL_LIBS_1_8_FOUND)
  find_package(PCL 1.8 REQUIRED)
  set(PCL_LIBRARIES "pcl_io-1.8 pcl_filters-1.8 pcl_segmentation-1.8 pcl_common-1.8 pcl_visualization-1.8")
endif()

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

### libpointmatcher (as a submodule)
#message(STATUS "Looking for pointmatcher")
#find_path(pointmatcher_INCLUDE_DIRS NAMES pointmatcher/IO.h pointmatcher/Parametrizable.h
#                                          pointmatcher/PointMatcher.h pointmatcher/#PointMatcherPrivate.h
#                                          pointmatcher/Registrar.h pointmatcher/Timer.h]
#            NO_DEFAULT_PATH PATHS /home/edbot/rpg-navigation/software/libpointmatcher
#)
#find_library(pointmatcher_LIBRARIES NAMES pointmatcher NO_DEFAULT_PATH PATHS
#            /home/edbot/rpg-navigation/software/libpointmatcher/build
#)
#if(pointmatcher_LIBRARIES AND pointmatcher_INCLUDE_DIRS)
#        include_directories(${pointmatcher_INCLUDE_DIRS})
#        message("-- pointmatcher found.")
#else(pointmatcher_LIBRARIES AND pointmatcher_INCLUDE_DIRS)
#        message("-- pointmatcher not found.")
#endif(pointmatcher_LIBRARIES AND pointmatcher_INCLUDE_DIRS)
#message(${pointmatcher_INCLUDE_DIRS})
#message(${pointmatcher_LIBRARIES})

############## registration library
add_library(aicpRegistration SHARED impl/pointmatcher_registration.cpp impl/aligned_sweeps_collection.cpp impl/sweep_scan.cpp impl/lidar_scan.cpp)
set_target_properties(aicpRegistration PROPERTIES SOVERSION 1)
target_link_libraries(aicpRegistration aicpPointmatcherUtils aicpCommonUtils aicpFilteringUtils pointmatcher)
pods_install_libraries(aicpRegistration)
pods_install_headers(include/common.hpp include/abstract_registrator.hpp impl/registration.hpp impl/pointmatcher_registration.hpp impl/aligned_sweeps_collection.hpp impl/sweep_scan.hpp impl/lidar_scan.hpp DESTINATION aicpRegistration)
pods_use_pkg_config_packages(aicpRegistration ${REQUIRED_PACKAGES} ${PCL_LIBRARIES} eigen3)

############### registration executable
add_executable(aicp-registration main.cpp)
target_link_libraries(aicp-registration boost_system aicpPointmatcherUtils aicpCommonUtils aicpDrawingUtils aicpRegistration aicpOverlap aicpClassification)
pods_use_pkg_config_packages(aicp-registration ${REQUIRED_PACKAGES} ${PCL_LIBRARIES} yaml-cpp eigen3 lcm pronto_vis bot2-lcmgl-client)
pods_install_executables(aicp-registration)

############### online registration executable
add_executable(aicp-registration-online online_app.cpp)
target_link_libraries(aicp-registration-online boost_system boost_filesystem cloud_accumulate aicpPointmatcherUtils aicpCommonUtils aicpDrawingUtils aicpRegistration aicpOverlap aicpClassification)
pods_use_pkg_config_packages(aicp-registration-online ${REQUIRED_PACKAGES} ${PCL_LIBRARIES} yaml-cpp eigen3 lcm pronto_vis bot2-lcmgl-client)
pods_install_executables(aicp-registration-online)



