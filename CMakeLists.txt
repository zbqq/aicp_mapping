cmake_minimum_required(VERSION 2.8.3)
project(aicp)

# get path to aicp base
add_definitions(-DPROJECTDIR=\"${CMAKE_CURRENT_SOURCE_DIR}\")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

# Find system dependencies (buildtime dependencies)
#find_package(Boost REQUIRED COMPONENTS system filesystem thread)
#find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(libpointmatcher REQUIRED)
find_package(PCL REQUIRED)
find_package(Octomap REQUIRED)
find_package(OpenCV 3.0 REQUIRED)
#find_package(yaml-cpp REQUIRED)

# Find catkin packages (buildtime dependencies)
find_package(catkin REQUIRED COMPONENTS
  #bot_lcmgl
  #octomap_utils
  #octomap_lcmtypes
  #pronto_vis
  #concise_args
  #cloud_accumulate
  #cv_bridge
  pointmatcher_ros
  pcl_ros
  octomap_ros
  #bot_core_lcmtypes
  #laser_geometry
  message_generation
)

# Service files to be built
add_service_files(FILES
  ProcessFile.srv
)
# Actually generate the language-specific service files
generate_messages(DEPENDENCIES)

#pkg_check_modules(yaml_cpp REQUIRED yaml-cpp)

## Add include directories
include_directories(
  include
  #${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  #${Eigen_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Add link directories
link_directories(
  #${Boost_LIBRARY_DIRS}
  #${Eigen_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES aicp_registration
  # CATKIN_DEPENDS message_runtime bot_lcmgl octomap_utils octomap_lcmtypes pronto_vis concise_args pointmatcher_ros pcl_ros
  CATKIN_DEPENDS message_runtime pointmatcher_ros pcl_ros octomap_ros
)


#find_package(VTK REQUIRED)
#include(${VTK_USE_FILE})

### pcl
#find_package(PkgConfig REQUIRED)

#pkg_check_modules(PCL_LIBS pcl_io-1.7)
#if(PCL_LIBS_FOUND)
#  find_package(PCL 1.7 REQUIRED)
#  set(PCL_LIBRARIES "pcl_io-1.7 pcl_filters-1.7 pcl_segmentation-1.7 pcl_common-1.7 pcl_visualization-1.7")
#endif()

#pkg_check_modules(PCL_LIBS_1_8 pcl_io-1.8)
#if(PCL_LIBS_1_8_FOUND)
#  find_package(PCL 1.8 REQUIRED)
#  set(PCL_LIBRARIES "pcl_io-1.8 pcl_filters-1.8 pcl_segmentation-1.8 pcl_common-1.8 pcl_visualization-1.8")
#endif()

#include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

add_subdirectory(src/utils)
add_subdirectory(src/registration)
add_subdirectory(src/overlap)
add_subdirectory(src/classification)
add_subdirectory(src/registration_apps)
#add_subdirectory(src/pcl)
#add_subdirectory(src/kitti)
#add_subdirectory(src/test)
