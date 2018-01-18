############### concatenate executable
add_executable (aicp-concatenate-clouds concatenate.cpp)
target_link_libraries (aicp-concatenate-clouds 
	${catkin_LIBRARIES} boost_system ${VTK_LIBRARIES})

############### create cube cloud executable
add_executable (aicp-create-cube-cloud create_cube_cloud.cpp)
target_link_libraries (aicp-create-cube-cloud
	${catkin_LIBRARIES} boost_system ${VTK_LIBRARIES})