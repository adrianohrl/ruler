cmake_minimum_required(VERSION 2.8.3)

project(alliance)

find_package(Boost REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  alliance_msgs
)

find_package(ruler REQUIRED COMPONENTS
  ruler_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    ${PROJECT_NAME}
    ruler
  CATKIN_DEPENDS
    roscpp
    alliance_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${ruler_INCLUDE_DIRS}
)

set(H_FILES
  include/alliance/acquiescence.h
  include/alliance/activity_suppression.h
  include/alliance/alliance.h
  include/alliance/behaviour_set.h
  include/alliance/impatience.h
  include/alliance/impatience_reset.h
  include/alliance/inter_communication.h
  include/alliance/layer.h
  include/alliance/motivational_behaviour.h
  include/alliance/robot.h
  include/alliance/sensor.h
  include/alliance/sensory_feedback.h
  include/alliance/task.h
  include/nodes/alliance_node.h
)

set(CPP_FILES
  src/alliance/acquiescence.cpp
  src/alliance/activity_suppression.cpp
  src/alliance/behaviour_set.cpp
  src/alliance/impatience.cpp
  src/alliance/impatience_reset.cpp
  src/alliance/inter_communication.cpp
  src/alliance/layer.cpp
  src/alliance/motivational_behaviour.cpp
  src/alliance/robot.cpp
  src/alliance/sensory_feedback.cpp
  src/alliance/task.cpp
  src/nodes/alliance_node.cpp
)

add_library(${PROJECT_NAME}
  ${CPP_FILES}
)

add_dependencies(${PROJECT_NAME}
  ${Boost_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${ruler_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${ruler_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node
  src/alliance_node.cpp
  ${CPP_FILES}
)

add_dependencies(${PROJECT_NAME}_node
  ${Boost_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${ruler_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_node
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${ruler_LIBRARIES}
)

catkin_add_gtest(${PROJECT_NAME}_utest
  test/utest.cpp
  ${CPP_FILES}
)

add_dependencies(${PROJECT_NAME}_utest
  ${Boost_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${ruler_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_utest
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${ruler_LIBRARIES}
)
