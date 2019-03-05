############### clasification library
add_library(aicpClassification SHARED svm.cpp)
set_target_properties(aicpClassification PROPERTIES SOVERSION 1)
target_link_libraries(aicpClassification ${OpenCV_LIBS})

############### classifier main executable
add_executable(aicp-classification main.cpp)
target_link_libraries(aicp-classification aicpClassification aicpCommonUtils yaml-cpp ${OpenCV_LIBS} ${libpointmatcher_LIBRARIES} ${PCL_LIBRARIES})

############### classifier example executable
add_executable(aicp-classification-example example.cpp)
target_link_libraries(aicp-classification-example aicpClassification aicpCommonUtils yaml-cpp ${OpenCV_LIBS} ${libpointmatcher_LIBRARIES} ${PCL_LIBRARIES})
