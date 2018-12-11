############## registration library
add_library(aicpRegistration SHARED pointmatcher_registration.cpp aligned_cloud.cpp aligned_clouds_graph.cpp)
set_target_properties(aicpRegistration PROPERTIES SOVERSION 1)
