set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(aicp)

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
set(PCL_LIBRARIES pcl_io-1.7 pcl_common-1.7 pcl_filters-1.7 pcl_features-1.7 pcl_io-1.7 pcl_surface-1.7 pcl_segmentation-1.7 pcl_visualization-1.7 pcl_search-1.7 pcl_kdtree-1.7 pcl_keypoints-1.7)

message(STATUS "Looking for pointmatcher")
find_path(pointmatcher_INCLUDE_DIRS NAMES pointmatcher/IO.h pointmatcher/Parametrizable.h
                                          pointmatcher/PointMatcher.h pointmatcher/PointMatcherPrivate.h
                                          pointmatcher/Registrar.h pointmatcher/Timer.h]
            NO_DEFAULT_PATH PATHS /home/edbot/rpg-navigation/software/libpointmatcher
)
find_library(pointmatcher_LIBRARIES NAMES pointmatcher NO_DEFAULT_PATH PATHS
            /home/edbot/rpg-navigation/software/libpointmatcher/build
)
if(pointmatcher_LIBRARIES AND pointmatcher_INCLUDE_DIRS)
        include_directories(${pointmatcher_INCLUDE_DIRS})
        message("-- pointmatcher found.")
else(pointmatcher_LIBRARIES AND pointmatcher_INCLUDE_DIRS)
        message("-- pointmatcher not found.")
endif(pointmatcher_LIBRARIES AND pointmatcher_INCLUDE_DIRS)
message(${pointmatcher_INCLUDE_DIRS})
message(${pointmatcher_LIBRARIES})

############## registration library
add_library(aicpRegistration SHARED impl/pointmatcher_registration.cpp impl/mixed_test_routine.cpp impl/aligned_sweeps_collection.cpp impl/sweep_scan.cpp impl/lidar_scan.cpp)
set_target_properties(aicpRegistration PROPERTIES SOVERSION 1)
target_link_libraries(aicpRegistration pointmatcherUtils commonUtils filteringUtils ${pointmatcher_LIBRARIES})
pods_install_libraries(aicpRegistration)
pods_install_headers(include/common.hpp include/abstract_registrator.hpp impl/registration.hpp impl/pointmatcher_registration.hpp impl/mixed_test_routine.hpp impl/aligned_sweeps_collection.hpp impl/sweep_scan.hpp impl/lidar_scan.hpp DESTINATION aicpRegistration)
pods_use_pkg_config_packages(aicpRegistration ${REQUIRED_PACKAGES} ${PCL_LIBRARIES} eigen3)

############### registration executable
add_executable(aicp-registration main.cpp)
target_link_libraries(aicp-registration boost_system pointmatcherUtils commonUtils drawingUtils aicpRegistration aicpOverlap aicpClassification)
pods_use_pkg_config_packages(aicp-registration ${REQUIRED_PACKAGES} ${PCL_LIBRARIES} yaml-cpp eigen3 lcm pronto_vis bot2-lcmgl-client)
pods_install_executables(aicp-registration)

############### online registration executable
add_executable(aicp-registration-online online_app.cpp)
target_link_libraries(aicp-registration-online boost_system boost_filesystem cloud_accumulate pointmatcherUtils commonUtils drawingUtils aicpRegistration aicpOverlap aicpClassification)
pods_use_pkg_config_packages(aicp-registration-online ${REQUIRED_PACKAGES} ${PCL_LIBRARIES} yaml-cpp eigen3 lcm pronto_vis bot2-lcmgl-client)
pods_install_executables(aicp-registration-online)



