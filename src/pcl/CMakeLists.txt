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

############## pcl_aicp library (functions defined in 1.7+)
add_library(pcl_aicp SHARED features/impl/moment_of_inertia_estimation.cpp)
set_target_properties(pcl_aicp PROPERTIES SOVERSION 1)
target_link_libraries(pcl_aicp)
pods_install_libraries(pcl_aicp)
pods_install_headers(features/impl/moment_of_inertia_estimation.hpp features/moment_of_inertia_estimation.h DESTINATION pcl_aicp)
pods_use_pkg_config_packages(pcl_aicp ${REQUIRED_PACKAGES} ${PCL_LIBRARIES})
pods_install_pkg_config_file(pointmatcherUtils
  LIBS
  REQUIRES eigen3
  VERSION 0.0.1)

############### test executable
add_executable(pcl_aicp_inertia_estimation main.cpp)
target_link_libraries(pcl_aicp_inertia_estimation boost_system boost_thread ${VTK_LIBRARIES} pcl_aicp)
pods_use_pkg_config_packages(pcl_aicp_inertia_estimation ${REQUIRED_PACKAGES} ${PCL_LIBRARIES})
pods_install_executables(pcl_aicp_inertia_estimation)
