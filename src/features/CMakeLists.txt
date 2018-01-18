############## pcl_aicp library (functions defined in 1.7+)
#add_library(pcl_aicp SHARED features/impl/moment_of_inertia_estimation.cpp)
#set_target_properties(pcl_aicp PROPERTIES SOVERSION 1)
#target_link_libraries(pcl_aicp)
#pods_install_libraries(pcl_aicp)
#pods_install_headers(features/impl/moment_of_inertia_estimation.hpp features/moment_of_inertia_estimation.h DESTINATION pcl_aicp)
#pods_use_pkg_config_packages(pcl_aicp ${REQUIRED_PACKAGES} ${PCL_LIBRARIES})
#pods_install_pkg_config_file(aicpPointmatcherUtils
#  LIBS
#  REQUIRES eigen3
#  VERSION 0.0.1)
