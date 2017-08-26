set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(aicp)

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
set(PCL_LIBRARIES pcl_io-1.7 pcl_common-1.7 pcl_filters-1.7 pcl_features-1.7 pcl_io-1.7 pcl_surface-1.7 pcl_segmentation-1.7 pcl_visualization-1.7 pcl_search-1.7 pcl_kdtree-1.7 pcl_keypoints-1.7)

############### clasification library
add_library(aicpClassification SHARED impl/svm.cpp)
set_target_properties(aicpClassification PROPERTIES SOVERSION 1)
pods_install_libraries(aicpClassification)
pods_install_headers( include/common.hpp include/abstract_classification.hpp impl/classification.hpp impl/svm.hpp DESTINATION aicpClassification)
pods_use_pkg_config_packages(aicpClassification ${PCL_LIBRARIES} ${REQUIRED_PACKAGES} opencv)

############### classifier main executable
add_executable(aicp-classification main.cpp)
target_link_libraries(aicp-classification aicpClassification commonUtils boost_system ${VTK_LIBRARIES})
pods_use_pkg_config_packages(aicp-classification ${PCL_LIBRARIES} yaml-cpp)
pods_install_executables(aicp-classification)

############### classifier train executable
#add_executable(dift-classifier-train train.cpp)
#target_link_libraries(dift-classifier-train diftClassification boost_system ${VTK_LIBRARIES})
#pods_use_pkg_config_packages(dift-classifier-train ${PCL_LIBRARIES} yaml-cpp)
#pods_install_executables(dift-classifier-train )

############### classifier test executable
#add_executable(dift-classifier-test test.cpp impl/random_forest.cpp)
#target_link_libraries(dift-classifier-test diftClassification boost_system ${VTK_LIBRARIES})
#pods_use_pkg_config_packages(dift-classifier-test ${PCL_LIBRARIES} yaml-cpp)
#pods_install_executables(dift-classifier-test )

############### classifier example executable
add_executable(aicp-classification-example example.cpp)
target_link_libraries(aicp-classification-example aicpClassification commonUtils boost_system ${VTK_LIBRARIES})
pods_use_pkg_config_packages(aicp-classification-example ${PCL_LIBRARIES} yaml-cpp)
pods_install_executables(aicp-classification-example)
