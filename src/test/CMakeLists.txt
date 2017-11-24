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


###############
add_executable(aicp-test test_aicp.cpp)
target_link_libraries(aicp-test boost_system aicpPointmatcherUtils aicpCommonUtils aicpDrawingUtils aicpRegistration aicpOverlap aicpClassification)
pods_use_pkg_config_packages(aicp-test ${REQUIRED_PACKAGES} ${PCL_LIBRARIES} yaml-cpp eigen3 lcm pronto_vis bot2-lcmgl-client)
pods_install_executables(aicp-test)

###############
add_executable(color-octomap-server color-octomap-server.cpp)
pods_use_pkg_config_packages(color-octomap-server octomap-util)
pods_install_executables(color-octomap-server)

###############
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})
add_executable(convert-vtk-to-octree-formats convert-vtk-to-octree-formats.cpp)
pods_use_pkg_config_packages(convert-vtk-to-octree-formats path-util ${REQUIRED_PACKAGES} ${PCL_IO_PKG_CONFIG}
    pronto_vis eigen3 bot2-frames octomap octomap-util)
target_link_libraries(convert-vtk-to-octree-formats  ${VTK_LIBRARIES})
target_link_libraries(convert-vtk-to-octree-formats lcm pronto_math  
                        boost_system boost_thread)
pods_install_executables(convert-vtk-to-octree-formats)

###############
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations -Wreturn-type -Wuninitialized -Wunused-variable ")
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})
add_executable(publish-vtk publish-vtk.cpp)
pods_use_pkg_config_packages(publish-vtk path-util   eigen3  )
target_link_libraries(publish-vtk lcm boost_system boost_thread ${VTK_LIBRARIES})
pods_install_executables(publish-vtk)

###############
add_executable(collections-demo collections-demo.cpp)
pods_use_pkg_config_packages(collections-demo path-util   eigen3  pronto_vis)
target_link_libraries(collections-demo lcm boost_system boost_thread)
pods_install_executables(collections-demo)
