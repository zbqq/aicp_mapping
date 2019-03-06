###########
include_directories(
  include
)

############### registration executable
#add_executable(aicp-registration main.cpp)
#target_link_libraries(aicp-registration ${catkin_LIBRARIES}
#                                        boost_system
#                                        yaml-cpp
#                                        aicpPointmatcherUtils
#                                        aicpFilteringUtils
#                                        aicpCommonUtils
#                                        aicpDrawingUtils
#                                        aicpRegistration
#                                        aicpOverlap
#                                        aicpClassification)

add_library(registration_apps app.cpp
                              yaml_configurator.cpp)
target_link_libraries(registration_apps yaml-cpp
                                        #boost_system
                                        #boost_filesystem
                                        aicpPointmatcherUtils
                                        aicpFilteringUtils
                                        aicpCommonUtils
                                        #aicpDrawingUtils
                                        aicpRegistration
                                        aicpOverlap
                                        aicpClassification)

############### online registration executable
#add_executable(aicp-lcm-online lcm_online_app.cpp
#                               app_lcm.cpp
#                               visualizer_lcm.cpp)
#target_link_libraries(aicp-lcm-online registration_apps)

add_executable(aicp_ros_online ros_online_app.cpp
                               app_ros.cpp
                               velodyne_accumulator.cpp
                               visualizer_ros.cpp)
target_link_libraries(aicp_ros_online registration_apps)
