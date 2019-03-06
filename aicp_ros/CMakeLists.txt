cmake_minimum_required(VERSION 2.8.3)

project(aicp_ros)
add_compile_options(-std=c++11)

# Find system dependencies (buildtime dependencies)
find_package(catkin REQUIRED COMPONENTS aicp_core
                                        aicp_srv
                                        roscpp
                                        eigen_conversions
                                        pcl_conversions
                                        tf_conversions
                                        std_msgs
                                        sensor_msgs
                                        geometry_msgs
                                        nav_msgs
                                        )
find_package(PCL REQUIRED)
find_package(OpenCV 3.0 REQUIRED)

# Add include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS aicp_core
                 aicp_srv
                 roscpp
                 pcl_conversions
                 tf_conversions
                 std_msgs
                 sensor_msgs
                 geometry_msgs
                 nav_msgs
                 )
  

add_library(${PROJECT_NAME} src/app_ros.cpp
                            src/visualizer_ros.cpp
                            src/velodyne_accumulator.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})


#add_executable(${PROJECT_NAME}_node src/pronto_ros_node.cpp)
#target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES})



#############
## Install ##
#############


## Mark executables and/or libraries for installation
#install(TARGETS ${PROJECT_NAME} #${PROJECT_NAME}_node
#         ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#         LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#         RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark cpp header files for installation
#install(DIRECTORY include/${PROJECT_NAME}/
#        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#        FILES_MATCHING PATTERN "*.hpp")
  

#add_subdirectory(src/registration_apps)
#add_subdirectory(src/pcl)
#add_subdirectory(src/kitti)
#add_subdirectory(src/test)
