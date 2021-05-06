cmake_minimum_required(VERSION 3.0.2)
project(bobi_control)

add_compile_options(-std=c++11)


# file(GLOB SOURCES
#     src/*.cpp
# )
# list(FILTER SOURCES EXCLUDE REGEX "fishbot_interface_node.cpp")

find_package(
    catkin REQUIRED COMPONENTS
    roscpp
    rospy
    geometry_msgs
    sensor_msgs
    bobi_msgs
    dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  config/VelocityController.cfg
  config/PositionController.cfg
)

catkin_package(
  CATKIN_DEPENDS bobi_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# message(STATUS ${SOURCES})

add_executable(velocity_controller_node src/velocity_controller_node.cpp)
target_link_libraries(velocity_controller_node ${catkin_LIBRARIES})
add_dependencies(velocity_controller_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg bobi_msgs_generate_messages_cpp)

add_executable(position_controller_node src/position_controller_node.cpp)
target_link_libraries(position_controller_node ${catkin_LIBRARIES})
add_dependencies(position_controller_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg bobi_msgs_generate_messages_cpp)
