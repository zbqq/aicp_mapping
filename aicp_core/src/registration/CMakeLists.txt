############## registration library
add_library(aicpRegistration SHARED pointmatcher_registration.cpp aligned_cloud.cpp aligned_clouds_graph.cpp)
set_target_properties(aicpRegistration PROPERTIES SOVERSION 1)
target_link_libraries(aicpRegistration ${catkin_LIBRARIES})

install(TARGETS aicpRegistration
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/aicp_registration/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.hpp"
        PATTERN ".svn" EXCLUDE)
