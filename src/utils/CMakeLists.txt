add_definitions(-ggdb3 -std=c++0x)

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

find_package(PkgConfig REQUIRED)
### PCL LIBS
pkg_check_modules(PCL_LIBS pcl_io-1.7)
if(PCL_LIBS_FOUND)
  message("---- PCL LIBS was found (in utils)\n")
  set(PCL_LIBS "pcl_io-1.7 pcl_filters-1.7 pcl_segmentation-1.7 pcl_common-1.7 pcl_visualization-1.7")
endif()

pkg_check_modules(PCL_LIBS_1_8 pcl_io-1.8)
if(PCL_LIBS_1_8_FOUND)
  message("---- PCL LIBS_1_8 was found (in utils)\n")
  set(PCL_LIBS "pcl_io-1.8 pcl_filters-1.8 pcl_segmentation-1.8 pcl_common-1.8 pcl_visualization-1.8")
endif()

######################################################################
add_library(drawingUtils SHARED drawingUtils.cpp)
set_target_properties(drawingUtils PROPERTIES SOVERSION 1)
pods_install_libraries(drawingUtils)
pods_install_headers( drawingUtils.hpp
              DESTINATION drawingUtils)
pods_use_pkg_config_packages(drawingUtils eigen3
  lcm    bot2-frames icp-registration
  lcmtypes_bot2-core ${REQUIRED_PACKAGES} ${PCL_LIBS})
pods_install_pkg_config_file(drawingUtils 
  LIBS 
  REQUIRES eigen3 
  VERSION 0.0.1)

add_library(timingUtils SHARED timingUtils.cpp)
set_target_properties(timingUtils PROPERTIES SOVERSION 1)
pods_install_libraries(timingUtils)
pods_install_headers( timingUtils.hpp
              DESTINATION timingUtils)
pods_use_pkg_config_packages(timingUtils eigen3
  lcm    bot2-frames ${REQUIRED_PACKAGES})
pods_install_pkg_config_file(timingUtils 
  LIBS 
  REQUIRES eigen3 
  VERSION 0.0.1)

add_library(filteringUtils SHARED filteringUtils.cpp)
set_target_properties(filteringUtils PROPERTIES SOVERSION 1)
pods_install_libraries(filteringUtils)
pods_install_headers( filteringUtils.hpp
              DESTINATION filteringUtils)
pods_use_pkg_config_packages(filteringUtils eigen3
  lcm    bot2-frames icp-registration
  lcmtypes_bot2-core ${REQUIRED_PACKAGES} ${PCL_LIBS})
pods_install_pkg_config_file(filteringUtils
  LIBS
  REQUIRES eigen3
  VERSION 0.0.1)
