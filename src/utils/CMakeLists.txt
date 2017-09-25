add_definitions(-ggdb3 -std=c++0x)

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

find_package(PkgConfig REQUIRED)
### PCL IO
pkg_check_modules(PCL_IO pcl_io-1.7)
if(PCL_IO_FOUND)
  message("---- PCL IO was found (in utils)\n")
  set(PCL_IO_PKG_CONFIG "pcl_io-1.7")
  set(USE_PRONTO_VIS_PCL TRUE)
endif()

pkg_check_modules(PCL_IO_1_8 pcl_io-1.8)
if(PCL_IO_1_8_FOUND)
  message("---- PCL IO_1_8 was found (in utils)\n")
  set(PCL_IO_PKG_CONFIG "pcl_io-1.8")
  set(USE_PRONTO_VIS_PCL TRUE)
endif()
### PCL FILTERS
pkg_check_modules(PCL_FILTERS pcl_filters-1.7)
if(PCL_FILTERS_FOUND)
  message("---- PCL FILTERS was found (in utils)\n")
  set(PCL_FILTERS_PKG_CONFIG "pcl_filters-1.7")
  set(USE_PRONTO_VIS_PCL TRUE)
endif()

pkg_check_modules(PCL_FILTERS_1_8 pcl_filters-1.8)
if(PCL_FILTERS_1_8_FOUND)
  message("---- PCL FILTERS_1_8 was found (in utils)\n")
  set(PCL_FILTERS_PKG_CONFIG "pcl_filters-1.8")
  set(USE_PRONTO_VIS_PCL TRUE)
endif()
### PCL SEGMENTATION
pkg_check_modules(PCL_SEGM pcl_segmentation-1.7)
if(PCL_SEGM_FOUND)
  message("---- PCL SEGM was found (in utils)\n")
  set(PCL_SEGM_PKG_CONFIG "pcl_segmentation-1.7")
  set(USE_PRONTO_VIS_PCL TRUE)
endif()

pkg_check_modules(PCL_SEGM_1_8 pcl_segmentation-1.8)
if(PCL_SEGM_1_8_FOUND)
  message("---- PCL SEGM_1_8 was found (in utils)\n")
  set(PCL_SEGM_PKG_CONFIG "pcl_segmentation-1.8")
  set(USE_PRONTO_VIS_PCL TRUE)
endif()
### PCL COMMON
pkg_check_modules(PCL_COMMON pcl_common-1.7)
if(PCL_COMMON_FOUND)
  message("---- PCL COMMON was found (in utils)\n")
  set(PCL_COMMON_PKG_CONFIG "pcl_common-1.7")
  set(USE_PRONTO_VIS_PCL TRUE)
endif()

pkg_check_modules(PCL_COMMON_1_8 pcl_common-1.8)
if(PCL_COMMON_1_8_FOUND)
  message("---- PCL COMMON_1_8 was found (in utils)\n")
  set(PCL_COMMON_PKG_CONFIG "pcl_common-1.8")
  set(USE_PRONTO_VIS_PCL TRUE)
endif()
### PCL VISUALIZATION
pkg_check_modules(PCL_VIS pcl_visualization-1.7)
if(PCL_VIS_FOUND)
  message("---- PCL VIS was found (in utils)\n")
  set(PCL_VIS_PKG_CONFIG "pcl_visualization-1.7")
  set(USE_PRONTO_VIS_PCL TRUE)
endif()

pkg_check_modules(PCL_VIS_1_8 pcl_visualization-1.8)
if(PCL_VIS_1_8_FOUND)
  message("---- PCL VIS_1_8 was found (in utils)\n")
  set(PCL_VIS_PKG_CONFIG "pcl_visualization-1.8")
  set(USE_PRONTO_VIS_PCL TRUE)
endif()

######################################################################
add_library(drawingUtils SHARED drawingUtils.cpp)
set_target_properties(drawingUtils PROPERTIES SOVERSION 1)
pods_install_libraries(drawingUtils)
pods_install_headers( drawingUtils.hpp
              DESTINATION drawingUtils)
pods_use_pkg_config_packages(drawingUtils eigen3
  lcm    bot2-frames icp-registration
  lcmtypes_bot2-core ${REQUIRED_PACKAGES} ${PCL_IO_PKG_CONFIG}
  ${FK_PACKAGES})
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
  lcmtypes_bot2-core ${REQUIRED_PACKAGES} ${PCL_IO_PKG_CONFIG} ${PCL_FILTERS_PKG_CONFIG} ${PCL_SEGM_PKG_CONFIG} ${PCL_COMMON_PKG_CONFIG} ${PCL_VIS_PKG_CONFIG} ${FK_PACKAGES})
pods_install_pkg_config_file(filteringUtils
  LIBS
  REQUIRES eigen3
  VERSION 0.0.1)
