set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

#cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
#project(aicp)

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

############### clasification library
add_library(aicpClassification SHARED svm.cpp)
set_target_properties(aicpClassification PROPERTIES SOVERSION 1)
#pods_install_libraries(aicpClassification)
#pods_install_headers( include/common.hpp include/abstract_classification.hpp impl/classification.hpp impl/svm.hpp DESTINATION aicpClassification)
#pods_use_pkg_config_packages(aicpClassification ${PCL_LIBRARIES} ${REQUIRED_PACKAGES} opencv)

############### classifier main executable
add_executable(aicp-classification main.cpp)
target_link_libraries(aicp-classification ${catkin_LIBRARIES} aicpClassification  aicpCommonUtils boost_system ${VTK_LIBRARIES} yaml-cpp ${OpenCV_LIBS})
#pods_use_pkg_config_packages(aicp-classification ${PCL_LIBRARIES} yaml-cpp)
#pods_install_executables(aicp-classification)

############### classifier example executable
message(STATUS "=====================")
message(STATUS "=====================")
message(STATUS "opencv " ${OpenCV_LIBS})

add_executable(aicp-classification-example example.cpp)
target_link_libraries(aicp-classification-example ${catkin_LIBRARIES} aicpClassification aicpCommonUtils boost_system ${VTK_LIBRARIES} yaml-cpp ${OpenCV_LIBS})
#pods_use_pkg_config_packages(aicp-classification-example ${PCL_LIBRARIES} yaml-cpp)
#pods_install_executables(aicp-classification-example)
