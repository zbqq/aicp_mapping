#############
# PCL Tools #
#############
############### remove ground
add_executable (aicp_ground_removal pcl_ground_removal.cpp)
target_link_libraries (aicp_ground_removal ${catkin_LIBRARIES} ${PCL_LIBRARIES})

#######################
# KITTI Dataset Tools #
#######################
############## kitti visualize poses executable
add_executable (aicp_kitti_trajectory kitti_trajectory_from_raw_gps.cpp)
target_link_libraries (aicp_kitti_trajectory ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

############## kitti evaluate
add_executable (aicp_kitti_evaluate kitti_devkit/evaluate_odometry.cpp kitti_devkit/matrix.cpp)
target_link_libraries (aicp_kitti_evaluate ${catkin_LIBRARIES})
