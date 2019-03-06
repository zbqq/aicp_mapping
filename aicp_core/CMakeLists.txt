cmake_minimum_required(VERSION 2.8.3)

project(aicp_core)
add_compile_options(-std=c++11)
set(AICP_CORE_LIB ${PROJECT_NAME})

# Find system dependencies (buildtime dependencies)
find_package(catkin REQUIRED)
find_package(libpointmatcher REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(octomap REQUIRED)
find_package(OpenCV 3.0 REQUIRED)

# Add include directories
include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${AICP_CORE_LIB}
  CATKIN_DEPENDS)
 
########################  
# registration library #
########################
add_library(aicpRegistration SHARED src/registration/pointmatcher_registration.cpp
                                    src/registration/aligned_cloud.cpp
                                    src/registration/aligned_clouds_graph.cpp)
target_link_libraries(aicpRegistration ${libpointmatcher_LIBRARIES}
                                       ${PCL_LIBRARIES}
                                       ${Eigen3_LIBRARIES})

##########################
# octree-overlap library #
##########################
add_library(aicpOverlap SHARED src/overlap/octrees_overlap.cpp)
target_link_libraries(aicpOverlap ${PCL_LIBRARIES}
                                  ${OCTOMAP_LIBRARIES})


#########################
# clasification library #
#########################
add_library(aicpClassification SHARED src/classification/svm.cpp)
target_link_libraries(aicpClassification ${Eigen3_LIBRARIES}
                                         ${OpenCV_LIBS})


#########################
# utils library #
#########################
add_library(aicpUtils SHARED src/utils/common.cpp
                             src/utils/timing.cpp
                             src/utils/cloudIO.cpp
                             src/utils/fileIO.cpp
                             src/utils/icpMonitor.cpp
                             src/utils/filteringUtils.cpp)
target_link_libraries(aicpUtils ${libpointmatcher_LIBRARIES}
                                ${PCL_LIBRARIES}
                                ${Eigen3_LIBRARIES})
                                      
#####################  
# aicp_core library #
#####################
add_library(${AICP_CORE_LIB} src/registration/app.cpp
                             src/registration/yaml_configurator.cpp)
target_link_libraries(${AICP_CORE_LIB} yaml-cpp
                                       aicpRegistration
                                       aicpOverlap
                                       aicpClassification
                                       aicpUtils)
                                       
                                       
# TODO: Unit test
#catkin_add_gtest(aicp_test src/test/imu_module_test.cpp)
#target_link_libraries(aicp_test ${catkin_LIBRARIES})


# Install libraries
install(TARGETS ${TARGETS}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/aicp_registration/
                  include/aicp_overlap/
                  include/aicp_classification/
                  include/aicp_filtering_utils/
                  include/aicp_common_utils/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.hpp"
        PATTERN ".svn" EXCLUDE)

# TODO: 
#add_subdirectory(src/test)
#add_subdirectory(src/pcl)
#add_subdirectory(src/kitti)
