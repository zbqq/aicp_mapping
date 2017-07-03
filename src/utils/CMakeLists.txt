add_definitions(-ggdb3 -std=c++0x)

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
set(PCL_LIBRARIES pcl_io-1.7 pcl_common-1.7 pcl_filters-1.7 pcl_features-1.7 pcl_io-1.7 pcl_surface-1.7 pcl_segmentation-1.7 pcl_visualization-1.7 pcl_search-1.7 pcl_kdtree-1.7 pcl_keypoints-1.7)

find_package(libpointmatcher REQUIRED)
include_directories(${CMAKE_CURRENT_BINARY_DIR} ${libpointmatcher_INCLUDE_DIRS})

##############
add_library(pointmatcherUtils SHARED icpMonitor.cpp)
set_target_properties(pointmatcherUtils PROPERTIES SOVERSION 1)
target_link_libraries(pointmatcherUtils ${VTK_LIBRARIES} ${libpointmatcher_LIBRARIES})
pods_install_libraries(pointmatcherUtils)
pods_install_headers(icpMonitor.h DESTINATION pointmatcherUtils)
pods_use_pkg_config_packages(pointmatcherUtils ${REQUIRED_PACKAGES} ${PCL_LIBRARIES})
pods_install_pkg_config_file(pointmatcherUtils 
  LIBS 
  REQUIRES eigen3 
  VERSION 0.0.1)

##############
add_library(drawingUtils SHARED drawingUtils.cpp)
set_target_properties(drawingUtils PROPERTIES SOVERSION 1)
pods_install_libraries(drawingUtils)
pods_install_headers( drawingUtils.hpp DESTINATION drawingUtils)
pods_use_pkg_config_packages(drawingUtils ${REQUIRED_PACKAGES} ${PCL_LIBRARIES})
pods_install_pkg_config_file(drawingUtils 
  LIBS 
  REQUIRES eigen3 
  VERSION 0.0.1)

##############
add_library(filteringUtils SHARED filteringUtils.cpp)
set_target_properties(filteringUtils PROPERTIES SOVERSION 1)
pods_install_libraries(filteringUtils)
pods_install_headers(filteringUtils.hpp DESTINATION filteringUtils)
pods_use_pkg_config_packages(filteringUtils ${REQUIRED_PACKAGES} ${PCL_LIBRARIES})
pods_install_pkg_config_file(filteringUtils 
  LIBS 
  REQUIRES eigen3 
  VERSION 0.0.1)

###############
add_library(commonUtils SHARED common.cpp timing.cpp cloudIO.cpp fileIO.cpp cloudIO.h vtkUtils.h fileIO.h)
set_target_properties(commonUtils PROPERTIES SOVERSION 1)
pods_install_libraries(commonUtils)
pods_install_headers(common.hpp timing.hpp cloudIO.h vtkUtils.h fileIO.h DESTINATION commonUtils)
pods_use_pkg_config_packages(commonUtils eigen3 ${REQUIRED_PACKAGES} ${PCL_LIBRARIES})
