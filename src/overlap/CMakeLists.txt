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

############## overlap from octrees library
add_library(aicpOverlap SHARED impl/octrees_overlap.cpp)
set_target_properties(aicpOverlap PROPERTIES SOVERSION 1)
target_link_libraries(aicpOverlap)
pods_install_libraries(aicpOverlap)
pods_install_headers(include/common.hpp include/abstract_overlapper.hpp impl/octrees_overlap.hpp impl/overlap.hpp DESTINATION aicpOverlap)
pods_use_pkg_config_packages(aicpOverlap eigen3 octomap ${REQUIRED_PACKAGES} ${PCL_LIBRARIES} octomap-util)
pods_install_pkg_config_file(aicpOverlap
    LIBS -laicp-overlap
    REQUIRES 
    VERSION 0.0.1)

############## overlap from octrees executable
add_executable (aicp-overlap main.cpp)
target_link_libraries(aicp-overlap lcm boost_system aicpOverlap commonUtils drawingUtils)
pods_use_pkg_config_packages(aicp-overlap eigen3 yaml-cpp lcm lcmtypes_bot2-core octomap ${REQUIRED_PACKAGES} ${PCL_LIBRARIES} octomap-util pronto_vis bot2-lcmgl-client)
pods_install_executables(aicp-overlap)




