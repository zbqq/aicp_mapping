##############################################
add_executable (testRegister2D testRegister2D.cpp)
target_link_libraries(testRegister2D boost_system)
pods_use_pkg_config_packages(testRegister2D eigen3
  lcm    bot2-frames
  lcmtypes_bot2-core  
  lidar-odom
  ${FK_PACKAGES})
pods_install_executables(testRegister2D)

###############
add_executable(color-octomap-server color-octomap-server.cpp)
pods_use_pkg_config_packages(color-octomap-server octomap-util)
pods_install_executables(color-octomap-server)

###############
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})
add_executable(convert-vtk-to-octree-formats convert-vtk-to-octree-formats.cpp)
pods_use_pkg_config_packages(convert-vtk-to-octree-formats path-util ${REQUIRED_PACKAGES} pcl_io-1.8
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

add_executable(test-aicp test_aicp.cpp)
target_link_libraries(test-aicp pointmatcher boost_system filteringUtils drawingUtils octrees-difference ${VTK_LIBRARIES})
pods_use_pkg_config_packages(test-aicp lcm bot2-lcmgl-client ${PCL_IO_PKG_CONFIG} icp-registration pronto_lcm pcl_features-1.8)
pods_install_executables(test-aicp)
