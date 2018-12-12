###############
#add_executable(aicp-test test_aicp.cpp)
#target_link_libraries(aicp-test 
#	boost_system yaml-cpp
#	aicpPointmatcherUtils aicpFilteringUtils
#	aicpCommonUtils aicpDrawingUtils 
#	aicpRegistration aicpOverlap aicpClassification)

###############
add_executable(color-octomap-server color-octomap-server.cpp)
target_link_libraries(color-octomap-server  ${catkin_LIBRARIES})

###############
add_executable(convert-vtk-to-octree-formats convert-vtk-to-octree-formats.cpp)
target_link_libraries(convert-vtk-to-octree-formats 
	${catkin_LIBRARIES} ${VTK_LIBRARIES} lcm  
	boost_system boost_thread)

###############
add_executable(publish-vtk publish-vtk.cpp)
target_link_libraries(publish-vtk lcm boost_system boost_thread ${VTK_LIBRARIES})

###############
add_executable(collections-demo collections-demo.cpp)
target_link_libraries(collections-demo ${catkin_LIBRARIES} lcm boost_system boost_thread)
