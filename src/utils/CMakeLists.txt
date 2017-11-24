add_definitions(-ggdb3 -std=c++0x)

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

### libpointmatcher (as a submodule)
#message(STATUS "Looking for pointmatcher")
#find_path(pointmatcher_INCLUDE_DIRS NAMES pointmatcher/IO.h pointmatcher/Parametrizable.h
#                                          pointmatcher/PointMatcher.h pointmatcher/#PointMatcherPrivate.h
#                                          pointmatcher/Registrar.h pointmatcher/Timer.h]
#            NO_DEFAULT_PATH PATHS /home/edbot/rpg-navigation/software/libpointmatcher
#)
#find_library(pointmatcher_LIBRARIES NAMES pointmatcher NO_DEFAULT_PATH PATHS
#            /home/edbot/rpg-navigation/software/libpointmatcher/build
#)
#if(pointmatcher_LIBRARIES AND pointmatcher_INCLUDE_DIRS)
#        include_directories(${pointmatcher_INCLUDE_DIRS})
#        message("-- pointmatcher found.")
#else(pointmatcher_LIBRARIES AND pointmatcher_INCLUDE_DIRS)
#        message("-- pointmatcher not found.")
#endif(pointmatcher_LIBRARIES AND pointmatcher_INCLUDE_DIRS)
#message(${pointmatcher_INCLUDE_DIRS})
#message(${pointmatcher_LIBRARIES})

##############
add_library(aicpPointmatcherUtils SHARED icpMonitor.cpp)
set_target_properties(aicpPointmatcherUtils PROPERTIES SOVERSION 1)
target_link_libraries(aicpPointmatcherUtils ${VTK_LIBRARIES})
pods_install_libraries(aicpPointmatcherUtils)
pods_install_headers(icpMonitor.h DESTINATION aicpPointmatcherUtils)
pods_use_pkg_config_packages(aicpPointmatcherUtils ${REQUIRED_PACKAGES} ${PCL_LIBRARIES})
pods_install_pkg_config_file(aicpPointmatcherUtils
  LIBS
  REQUIRES eigen3
  VERSION 0.0.1)

###############
add_library(aicpDrawingUtils SHARED drawingUtils.cpp)
set_target_properties(aicpDrawingUtils PROPERTIES SOVERSION 1)
target_link_libraries(aicpDrawingUtils ${VTK_LIBRARIES} octomap lcmtypes_octomap-utils bot2-core)
pods_install_libraries(aicpDrawingUtils)
pods_install_headers(drawingUtils.hpp DESTINATION aicpDrawingUtils)
pods_use_pkg_config_packages(aicpDrawingUtils ${REQUIRED_PACKAGES} ${PCL_LIBRARIES})
pods_install_pkg_config_file(aicpDrawingUtils
  LIBS
  REQUIRES eigen3
  VERSION 0.0.1)

##############
add_library(aicpFilteringUtils SHARED filteringUtils.cpp)
set_target_properties(aicpFilteringUtils PROPERTIES SOVERSION 1)
target_link_libraries(aicpFilteringUtils ${VTK_LIBRARIES})
pods_install_libraries(aicpFilteringUtils)
pods_install_headers(filteringUtils.hpp DESTINATION aicpFilteringUtils)
pods_use_pkg_config_packages(aicpFilteringUtils ${REQUIRED_PACKAGES} ${PCL_LIBRARIES})
pods_install_pkg_config_file(aicpFilteringUtils
  LIBS
  REQUIRES eigen3
  VERSION 0.0.1)

###############
add_library(aicpCommonUtils SHARED common.cpp timing.cpp cloudIO.cpp fileIO.cpp)
set_target_properties(aicpCommonUtils PROPERTIES SOVERSION 1)
target_link_libraries(aicpCommonUtils ${VTK_LIBRARIES} pointmatcher)
pods_install_libraries(aicpCommonUtils)
pods_install_headers(common.hpp timing.hpp cloudIO.h vtkUtils.h fileIO.h DESTINATION aicpCommonUtils)
pods_use_pkg_config_packages(aicpCommonUtils ${REQUIRED_PACKAGES} ${PCL_LIBRARIES})
pods_install_pkg_config_file(aicpCommonUtils
  LIBS
  REQUIRES eigen3
  VERSION 0.0.1)
