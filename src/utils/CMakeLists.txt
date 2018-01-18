find_package(libpointmatcher REQUIRED)

##############
add_library(aicpPointmatcherUtils SHARED icpMonitor.cpp)
set_target_properties(aicpPointmatcherUtils PROPERTIES SOVERSION 1)
target_link_libraries(aicpPointmatcherUtils  ${catkin_LIBRARIES} ${VTK_LIBRARIES})

###############
add_library(aicpDrawingUtils SHARED drawingUtils.cpp)
set_target_properties(aicpDrawingUtils PROPERTIES SOVERSION 1)
target_link_libraries(aicpDrawingUtils  ${catkin_LIBRARIES} ${VTK_LIBRARIES})

##############
add_library(aicpFilteringUtils SHARED filteringUtils.cpp)
set_target_properties(aicpFilteringUtils PROPERTIES SOVERSION 1)
target_link_libraries(aicpFilteringUtils ${catkin_LIBRARIES} ${VTK_LIBRARIES})

###############
add_library(aicpCommonUtils SHARED common.cpp timing.cpp cloudIO.cpp fileIO.cpp)
set_target_properties(aicpCommonUtils PROPERTIES SOVERSION 1)
target_link_libraries(aicpCommonUtils ${catkin_LIBRARIES} ${VTK_LIBRARIES} ${libpointmatcher_LIBRARIES})