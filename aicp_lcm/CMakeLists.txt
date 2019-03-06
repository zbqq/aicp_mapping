cmake_minimum_required(VERSION 2.8.3)

project(aicp_lcm)
add_compile_options(-std=c++11)

# Find system dependencies (buildtime dependencies)
find_package(catkin REQUIRED COMPONENTS aicp_core
                                        aicp_srv
                                        eigen_conversions
                                        pcl_conversions
                                        tf_conversions
                                        octomap_lcmtypes
                                        octomap_utils
                                        bot_core_lcmtypes
                                        pronto_vis
                                        cloud_accumulate
                                        concise_args)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS aicp_core
                 aicp_srv
                 eigen_conversions
                 pcl_conversions
                 tf_conversions
                 octomap_lcmtypes
                 octomap_utils
                 bot_core_lcmtypes
                 pronto_vis
                 cloud_accumulate
                 concise_args)

# Add include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME} SHARED src/app_lcm.cpp
                                   src/visualizer_lcm.cpp
                                   src/drawingUtils.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})


add_executable(${PROJECT_NAME}_node src/aicp_lcm_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}
                                           ${catkin_LIBRARIES}) 

############                                       
# Examples #
############
#add_executable(aicp_registration_main examples/registration_main.cpp)
#target_link_libraries(aicp_registration_main ${PROJECT_NAME}
#                                             ${catkin_LIBRARIES}
#                                             yaml-cpp)               
#add_executable(aicp_overlap_main examples/overlap_main.cpp)
#target_link_libraries(aicp_overlap_main ${PROJECT_NAME}
#                                        ${catkin_LIBRARIES}
#                                        yaml-cpp)
#add_executable(aicp_planar_laser_accumulate_main examples/planar_laser_accumulate.cpp)
#target_link_libraries(aicp_planar_laser_accumulate_main ${PROJECT_NAME}
#                                        ${catkin_LIBRARIES})
#add_executable(aicp_collections_main examples/collections_demo.cpp)
#target_link_libraries(aicp_collections_main ${PROJECT_NAME}
#                                        ${catkin_LIBRARIES})
#add_executable(aicp_color_octomap_main examples/color_octomap_server.cpp)
#target_link_libraries(aicp_color_octomap_main ${PROJECT_NAME}
#                                              ${catkin_LIBRARIES})
#add_executable(aicp_publish_vtk_main examples/publish_vtk.cpp)
#target_link_libraries(aicp_publish_vtk_main ${PROJECT_NAME}
#                                            ${catkin_LIBRARIES})
#add_executable(aicp_vtk_to_octree_main examples/vtk_to_octree.cpp)
#target_link_libraries(aicp_vtk_to_octree_main ${PROJECT_NAME}
#                                              ${catkin_LIBRARIES})


#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} #${PROJECT_NAME}_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.hpp")
  
