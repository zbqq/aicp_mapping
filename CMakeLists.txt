cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME registration)
include(cmake/pods.cmake)

# get path to registration base
add_definitions(-DCONFDIR=\"${CMAKE_CURRENT_SOURCE_DIR}\")

# create an executable, and make it public
add_subdirectory(src/registration)
add_subdirectory(src/sequential_registration)
add_subdirectory(src/analysis)
add_subdirectory(src/test)
add_subdirectory(src/utils)
add_subdirectory(src/octrees_difference)

pods_install_python_script(convert_robot_behavior python/convert_robot_behavior_type.py)
