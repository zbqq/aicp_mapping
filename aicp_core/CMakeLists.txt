cmake_minimum_required(VERSION 2.8.3)

project(aicp_core)
add_compile_options(-std=c++11)

# Find system dependencies (buildtime dependencies)
find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(libpointmatcher REQUIRED)
find_package(PCL REQUIRED)
find_package(octomap REQUIRED)
find_package(OpenCV 3.0 REQUIRED)

# Add include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

## Add link directories
#link_directories(
  #${Boost_LIBRARY_DIRS}
  #${Eigen_LIBRARY_DIRS}
#  ${PCL_LIBRARY_DIRS}
#)

catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES
  CATKIN_DEPENDS)

add_subdirectory(src/utils)
add_subdirectory(src/registration)
add_subdirectory(src/overlap)
add_subdirectory(src/classification)
#add_subdirectory(src/test)
#add_subdirectory(src/pcl)
#add_subdirectory(src/kitti)
