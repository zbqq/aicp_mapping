############### kitti visualize poses executable
add_executable (aicp-kitti-trajectory kitti_trajectory_from_raw_gps.cpp)
target_link_libraries (aicp-kitti-trajectory 
	${catkin_LIBRARIES} boost_system ${VTK_LIBRARIES})
	
############### kitti evaluate library
add_library(aicpKitti SHARED devkit/matrix.cpp)
set_target_properties(aicpKitti PROPERTIES SOVERSION 1)
target_link_libraries(aicpKitti ${OpenCV_LIBS})

############### kitti evaluate executable
add_executable (aicp-kitti-evaluate devkit/evaluate_odometry.cpp)
target_link_libraries (aicp-kitti-evaluate aicpKitti
	${catkin_LIBRARIES} boost_system)
