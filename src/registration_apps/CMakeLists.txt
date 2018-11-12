set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

### pcl
find_package(PkgConfig REQUIRED)

pkg_check_modules(PCL_LIBS pcl_io-1.7)
if(PCL_LIBS_FOUND)
  find_package(PCL 1.7 REQUIRED)
  set(PCL_LIBRARIES "pcl_io-1.7 pcl_filters-1.7 pcl_segmentation-1.7 pcl_common-1.7 pcl_visualization-1.7")
endif()

pkg_check_modules(PCL_LIBS_1_8 pcl_io-1.8)
if(PCL_LIBS_1_8_FOUND)
  find_package(PCL 1.8 REQUIRED)
  set(PCL_LIBRARIES "pcl_io-1.8 pcl_filters-1.8 pcl_segmentation-1.8 pcl_common-1.8 pcl_visualization-1.8")
endif()

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(include)
############### registration executable
add_executable(aicp-registration main.cpp)
target_link_libraries(aicp-registration ${catkin_LIBRARIES}
	boost_system yaml-cpp
	aicpPointmatcherUtils aicpFilteringUtils
	aicpCommonUtils aicpDrawingUtils aicpRegistration
	aicpOverlap aicpClassification)

add_library(registration_apps app.cpp
                              yaml_configurator.cpp)
target_link_libraries(registration_apps yaml-cpp
                                        boost_system
                                        boost_filesystem
                                        aicpPointmatcherUtils
                                        aicpFilteringUtils
                                        aicpCommonUtils
                                        #aicpDrawingUtils
                                        aicpRegistration
                                        aicpOverlap
                                        aicpClassification)

############### online registration executable
add_executable(aicp-registration-lcm-online lcm_online_app.cpp  app_lcm.cpp)
target_link_libraries(aicp-registration-lcm-online registration_apps)

add_executable(aicp-registration-ros-online ros_online_app.cpp app.cpp app_ros.cpp)
target_link_libraries(aicp-registration-ros-online registration_apps)
