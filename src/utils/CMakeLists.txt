add_definitions(-ggdb3 -std=c++0x)

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

add_library(drawingUtils SHARED drawingUtils.cpp)
set_target_properties(drawingUtils PROPERTIES SOVERSION 1)
pods_install_libraries(drawingUtils)
pods_install_headers( drawingUtils.hpp
              DESTINATION drawingUtils)
pods_use_pkg_config_packages(drawingUtils eigen3
  lcm    bot2-frames icp-registration drc-lcmtypes
  lcmtypes_bot2-core ${REQUIRED_PACKAGES} pcl_io-1.7
  ${FK_PACKAGES})
pods_install_pkg_config_file(drawingUtils 
  LIBS 
  REQUIRES eigen3 
  VERSION 0.0.1)
