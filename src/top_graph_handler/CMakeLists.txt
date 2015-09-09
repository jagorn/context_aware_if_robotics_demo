cmake_minimum_required(VERSION 2.8.3)
project(top_graph_handler)

find_package(catkin REQUIRED COMPONENTS
  genmsg
  message_generation
  roscpp
  rospy
  tf
  std_msgs
  adaptive_controller
)

find_package(OpenCV REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
  DEPENDS OpenCV
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(top_graph_handler src/top_graph_handler.cpp)
add_dependencies(top_graph_handler top_graph_handler_generate_messages_cpp)
add_dependencies(top_graph_handler ${catkin_EXPORTED_TARGETS})

target_link_libraries(top_graph_handler
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   top_graph_gen
 )
