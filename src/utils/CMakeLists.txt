add_definitions(-ggdb3 -std=c++0x)

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
set(PCL_LIBRARIES pcl_io-1.7 pcl_common-1.7 pcl_filters-1.7 pcl_features-1.7 pcl_io-1.7 pcl_surface-1.7 pcl_segmentation-1.7 pcl_visualization-1.7 pcl_search-1.7 pcl_kdtree-1.7 pcl_keypoints-1.7)

message(STATUS "Looking for pointmatcher")
find_path(pointmatcher_INCLUDE_DIRS NAMES pointmatcher/IO.h pointmatcher/Parametrizable.h
                                          pointmatcher/PointMatcher.h pointmatcher/PointMatcherPrivate.h
                                          pointmatcher/Registrar.h pointmatcher/Timer.h]
            NO_DEFAULT_PATH PATHS /home/edbot/rpg-navigation/software/libpointmatcher
)
find_library(pointmatcher_LIBRARIES NAMES pointmatcher NO_DEFAULT_PATH PATHS
            /home/edbot/rpg-navigation/software/libpointmatcher/build
)
if(pointmatcher_LIBRARIES AND pointmatcher_INCLUDE_DIRS)
        include_directories(${pointmatcher_INCLUDE_DIRS})
        message("-- pointmatcher found.")
else(pointmatcher_LIBRARIES AND pointmatcher_INCLUDE_DIRS)
        message("-- pointmatcher not found.")
endif(pointmatcher_LIBRARIES AND pointmatcher_INCLUDE_DIRS)
message(${pointmatcher_INCLUDE_DIRS})
message(${pointmatcher_LIBRARIES})

##############
add_library(pointmatcherUtils SHARED icpMonitor.cpp)
set_target_properties(pointmatcherUtils PROPERTIES SOVERSION 1)
target_link_libraries(pointmatcherUtils ${VTK_LIBRARIES})
pods_install_libraries(pointmatcherUtils)
pods_install_headers(icpMonitor.h DESTINATION pointmatcherUtils)
pods_use_pkg_config_packages(pointmatcherUtils ${REQUIRED_PACKAGES} ${PCL_LIBRARIES})
pods_install_pkg_config_file(pointmatcherUtils
  LIBS
  REQUIRES eigen3
  VERSION 0.0.1)

###############
add_library(drawingUtils SHARED drawingUtils.cpp)
set_target_properties(drawingUtils PROPERTIES SOVERSION 1)
target_link_libraries(drawingUtils ${VTK_LIBRARIES})
pods_install_libraries(drawingUtils)
pods_install_headers(drawingUtils.hpp DESTINATION drawingUtils)
pods_use_pkg_config_packages(drawingUtils ${REQUIRED_PACKAGES} ${PCL_LIBRARIES})
pods_install_pkg_config_file(drawingUtils
  LIBS
  REQUIRES eigen3
  VERSION 0.0.1)

##############
add_library(filteringUtils SHARED filteringUtils.cpp)
set_target_properties(filteringUtils PROPERTIES SOVERSION 1)
target_link_libraries(filteringUtils ${VTK_LIBRARIES} pcl_aicp)
pods_install_libraries(filteringUtils)
pods_install_headers(filteringUtils.hpp DESTINATION filteringUtils)
pods_use_pkg_config_packages(filteringUtils ${REQUIRED_PACKAGES} ${PCL_LIBRARIES})
pods_install_pkg_config_file(filteringUtils
  LIBS
  REQUIRES eigen3
  VERSION 0.0.1)

###############
add_library(commonUtils SHARED common.cpp timing.cpp cloudIO.cpp fileIO.cpp)
set_target_properties(commonUtils PROPERTIES SOVERSION 1)
target_link_libraries(commonUtils ${VTK_LIBRARIES})
pods_install_libraries(commonUtils)
pods_install_headers(common.hpp timing.hpp cloudIO.h vtkUtils.h fileIO.h DESTINATION commonUtils)
pods_use_pkg_config_packages(commonUtils ${REQUIRED_PACKAGES} ${PCL_LIBRARIES})
pods_install_pkg_config_file(commonUtils
  LIBS
  REQUIRES eigen3
  VERSION 0.0.1)
