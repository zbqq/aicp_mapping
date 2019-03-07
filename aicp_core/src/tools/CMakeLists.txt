#############
# PCL Tools #
#############
############### concatenate clouds
add_executable (aicp_concatenate_clouds concatenate_clouds.cpp)
target_link_libraries (aicp_concatenate_clouds ${PCL_LIBRARIES})

############### create cubic cloud
add_executable (aicp_create_cube_cloud create_cube_cloud.cpp)
target_link_libraries (aicp_create_cube_cloud ${PCL_LIBRARIES})
