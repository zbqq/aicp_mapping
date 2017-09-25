cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME aicp)
include(cmake/pods.cmake)

# get path to aicp base
add_definitions(-DPROJECTDIR=\"${CMAKE_CURRENT_SOURCE_DIR}\")

# create an executable, and make it public
add_subdirectory(src/registration)
add_subdirectory(src/utils)
add_subdirectory(src/pcl)
add_subdirectory(src/overlap)
add_subdirectory(src/classification)
#add_subdirectory(src/test) # TO BE UPDATED

pods_install_python_script(convert_robot_behavior python/convert_robot_behavior_type.py)
