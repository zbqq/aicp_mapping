############## overlap from octrees library
add_library(aicpOverlap SHARED octrees_overlap.cpp)
set_target_properties(aicpOverlap PROPERTIES SOVERSION 1)
#target_link_libraries(aicpOverlap)

############## overlap from octrees executable
#add_executable (aicp-overlap main.cpp)
#target_link_libraries(aicp-overlap lcm boost_system aicpOverlap aicpCommonUtils aicpDrawingUtils yaml-cpp)
