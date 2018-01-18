set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

#cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
#project(aicp)

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

############## pcl_aicp library (functions defined in 1.7+)
#add_library(pcl_aicp SHARED features/impl/moment_of_inertia_estimation.cpp)
#set_target_properties(pcl_aicp PROPERTIES SOVERSION 1)
#target_link_libraries(pcl_aicp)
#pods_install_libraries(pcl_aicp)
#pods_install_headers(features/impl/moment_of_inertia_estimation.hpp features/moment_of_inertia_estimation.h DESTINATION pcl_aicp)
#pods_use_pkg_config_packages(pcl_aicp ${REQUIRED_PACKAGES} ${PCL_LIBRARIES})
#pods_install_pkg_config_file(aicpPointmatcherUtils
#  LIBS
#  REQUIRES eigen3
#  VERSION 0.0.1)

############### test executable
#add_executable(pcl_aicp_inertia_estimation main.cpp)
#target_link_libraries(pcl_aicp_inertia_estimation boost_system boost_thread ${VTK_LIBRARIES} pcl_aicp)
#pods_use_pkg_config_packages(pcl_aicp_inertia_estimation ${REQUIRED_PACKAGES} ${PCL_LIBRARIES})
#pods_install_executables(pcl_aicp_inertia_estimation)

############### concatenate executable
add_executable (aicp-concatenate-clouds concatenate.cpp)
#pods_use_pkg_config_packages(aicp-concatenate-clouds ${REQUIRED_PACKAGES} ${PCL_LIBRARIES})
target_link_libraries (aicp-concatenate-clouds 
	${catkin_LIBRARIES} boost_system ${VTK_LIBRARIES})
#pods_install_executables(aicp-concatenate-clouds)

############### create cube cloud executable
add_executable (aicp-create-cube-cloud create_cube_cloud.cpp)
#pods_use_pkg_config_packages(aicp-create-cube-cloud ${REQUIRED_PACKAGES} ${PCL_LIBRARIES})
target_link_libraries (aicp-create-cube-cloud
	${catkin_LIBRARIES} boost_system ${VTK_LIBRARIES})
#pods_install_executables(aicp-create-cube-cloud)
