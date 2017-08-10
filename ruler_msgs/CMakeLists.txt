cmake_minimum_required(VERSION 2.8.3)

project(ruler_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  message_generation
  roscpp
  std_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
    Resource.msg
    Task.msg
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
    nav_msgs
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
    nav_msgs
    std_msgs
)
