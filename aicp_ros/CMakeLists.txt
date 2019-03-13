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
                                        nav_msgs)
                        
#find_package(aicp_core)
#message("++++++++++++++++++++++ ${aicp_core_LIBRARIES}") # contains all libraries and dependencies declated in catkin_package of aicp_core                                        

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS aicp_core
                 aicp_srv
                 roscpp
                 eigen_conversions
                 pcl_conversions
                 tf_conversions
                 std_msgs
                 sensor_msgs
                 geometry_msgs
                 nav_msgs)

# Add include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME} SHARED src/app_ros.cpp
                                   src/visualizer_ros.cpp
                                   src/velodyne_accumulator.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})


add_executable(${PROJECT_NAME}_node src/aicp_ros_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}
                                           ${catkin_LIBRARIES})                                           


#############                                       
# Unit test #
#############
#catkin_add_gtest(aicp_test_node test/aicp_test_node.cpp)
#add_executable(aicp_test_node test/aicp_test_node.cpp)
#target_link_libraries(aicp_test_node ${PROJECT_NAME}
#                                     ${catkin_LIBRARIES}
#                                     yaml-cpp)


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

