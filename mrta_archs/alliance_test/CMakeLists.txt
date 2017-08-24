cmake_minimum_required(VERSION 2.8.3)

project(alliance_test)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

find_package(alliance REQUIRED COMPONENTS
  alliance_msgs
  ruler
  ruler_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    alliance
    ${PROJECT_NAME}
    ruler
  CATKIN_DEPENDS
    alliance
    alliance_msgs
    roscpp
    ruler
    ruler_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${alliance_INCLUDE_DIRS}
)

set(H_FILES
)

set(CPP_FILES
)
