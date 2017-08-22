cmake_minimum_required(VERSION 2.8.3)

project(ruler_metrics)

find_package(Boost REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  pluginlib
  roscpp
)

find_package(ruler REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    ${PROJECT_NAME}
    ruler
  CATKIN_DEPENDS
    roscpp
    pluginlib
  DEPENDS Boost
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${ruler_INCLUDE_DIRS}
)

set(H_FILES
  include/ruler/battery_consumption_estimator.h
  include/ruler/displacement_estimator.h
  include/ruler/metrics_plugins.cpp
)

set(CPP_FILES
  src/ruler/battery_consumption_estimator.cpp
  src/ruler/displacement_estimator.cpp
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
