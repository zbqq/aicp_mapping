cmake_minimum_required(VERSION 2.8.3)

project(aicp_srv)
add_compile_options(-std=c++11)

# Find system dependencies (buildtime dependencies)
find_package(catkin REQUIRED COMPONENTS roscpp
                                        message_generation)

# Service files to be built
add_service_files(FILES
  ProcessFile.srv
)
generate_messages(DEPENDENCIES)

catkin_package(
  CATKIN_DEPENDS message_runtime
)
