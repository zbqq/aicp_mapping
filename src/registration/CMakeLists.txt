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
add_library(aicpRegistration SHARED impl/pointmatcher_registration.cpp impl/mixed_test_routine.cpp)
set_target_properties(aicpRegistration PROPERTIES SOVERSION 1)
target_link_libraries(aicpRegistration pointmatcherUtils commonUtils filteringUtils ${pointmatcher_LIBRARIES})
pods_install_libraries(aicpRegistration)
pods_install_headers(include/common.hpp include/abstract_registrator.hpp impl/registration.hpp impl/pointmatcher_registration.hpp impl/mixed_test_routine.hpp DESTINATION aicpRegistration)
pods_use_pkg_config_packages(aicpRegistration ${REQUIRED_PACKAGES} ${PCL_LIBRARIES} eigen3)

############### registration executable
add_executable(aicp-registration main.cpp)
target_link_libraries(aicp-registration boost_system pointmatcherUtils commonUtils aicpRegistration)
pods_use_pkg_config_packages(aicp-registration ${REQUIRED_PACKAGES} ${PCL_LIBRARIES} yaml-cpp eigen3)
pods_install_executables(aicp-registration)

###############
#add_executable(drc-lidar-accumulate lidar_accumulate.cpp clouds_io_utils.cpp)
#pods_use_pkg_config_packages(drc-lidar-accumulate icp-registration  bot2-frames laser-util
#    path-util ${REQUIRED_PACKAGES} pcl_io-1.7
#    pronto_vis   eigen3)
#target_link_libraries (drc-lidar-accumulate lcm pronto_math cloud_accumulate 
#                        boost_system boost_thread)
#pods_install_executables(drc-lidar-accumulate)

###############
#add_executable(drc-icp-plot icp_registration_app.cpp icp_3Dreg_and_plot.cpp)
#pods_use_pkg_config_packages(drc-icp-plot path-util ${REQUIRED_PACKAGES} pcl_io-1.7
#    pronto_vis eigen3 bot2-frames)
#target_link_libraries(drc-icp-plot icp-registration lcm pronto_math ${VTK_LIBRARIES}
#                        boost_system boost_thread)
#pods_install_executables(drc-icp-plot)

###############
#add_executable(drc-icp-plot-advanced icp_registration_advanced_app.cpp icp_3Dreg_and_plot.cpp)
#pods_use_pkg_config_packages(drc-icp-plot-advanced path-util ${REQUIRED_PACKAGES} pcl_io-1.7
#    pronto_vis eigen3 bot2-frames)
#target_link_libraries(drc-icp-plot-advanced icp-registration lcm pronto_math ${VTK_LIBRARIES}
#                        boost_system boost_thread)
#pods_install_executables(drc-icp-plot-advanced)


