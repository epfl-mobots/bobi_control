cmake_minimum_required(VERSION 3.0.2)
project(bobi_control)

add_compile_options(-std=c++17)

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

add_executable(velocity_controller_node src/velocity_controller_node.cpp)
target_link_libraries(velocity_controller_node ${catkin_LIBRARIES})
add_dependencies(velocity_controller_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg bobi_msgs_generate_messages_cpp)

add_executable(position_controller_node src/position_controller_node.cpp)
target_link_libraries(position_controller_node ${catkin_LIBRARIES})
add_dependencies(position_controller_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg bobi_msgs_generate_messages_cpp)

add_executable(circular_trajectory_node src/test/circular_trajectory_node.cpp)
target_link_libraries(circular_trajectory_node ${catkin_LIBRARIES})
add_dependencies(circular_trajectory_node ${catkin_EXPORTED_TARGETS} bobi_msgs_generate_messages_cpp)

add_executable(eights_node src/test/eights_node.cpp)
target_link_libraries(eights_node ${catkin_LIBRARIES})
add_dependencies(eights_node ${catkin_EXPORTED_TARGETS} bobi_msgs_generate_messages_cpp)

add_executable(straight_line_node src/test/straight_line_node.cpp)
target_link_libraries(straight_line_node ${catkin_LIBRARIES})
add_dependencies(straight_line_node ${catkin_EXPORTED_TARGETS} bobi_msgs_generate_messages_cpp)
