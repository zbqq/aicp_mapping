############### concatenate executable
add_executable (aicp-concatenate-clouds concatenate.cpp)
target_link_libraries (aicp-concatenate-clouds 
	${catkin_LIBRARIES} boost_system ${VTK_LIBRARIES})

############### create cube cloud executable
add_executable (aicp-create-cube-cloud create_cube_cloud.cpp)
target_link_libraries (aicp-create-cube-cloud
	${catkin_LIBRARIES} boost_system ${VTK_LIBRARIES})

############### convert kitti cloud to pcd executable
add_executable (aicp-kitti2pcd kitti2pcd.cpp)
target_link_libraries (aicp-kitti2pcd
	${catkin_LIBRARIES} boost_system ${VTK_LIBRARIES})

############### remove ground executable
add_executable (aicp-ground-removal pcl_ground_removal.cpp)
target_link_libraries (aicp-ground-removal
	${catkin_LIBRARIES} boost_system ${VTK_LIBRARIES})
