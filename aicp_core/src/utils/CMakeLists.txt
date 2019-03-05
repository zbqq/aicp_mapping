add_library(aicpPointmatcherUtils SHARED icpMonitor.cpp)
set_target_properties(aicpPointmatcherUtils PROPERTIES SOVERSION 1)
#target_link_libraries(aicpPointmatcherUtils)

add_library(aicpFilteringUtils SHARED filteringUtils.cpp)
set_target_properties(aicpFilteringUtils PROPERTIES SOVERSION 1)
#target_link_libraries(aicpFilteringUtils)

add_library(aicpCommonUtils SHARED common.cpp timing.cpp cloudIO.cpp fileIO.cpp)
set_target_properties(aicpCommonUtils PROPERTIES SOVERSION 1)
#target_link_libraries(aicpCommonUtils ${libpointmatcher_LIBRARIES} ${PCL_LIBRARIES})
