cmake_minimum_required(VERSION 2.8.3)

project(alliance)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  alliance_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    alliance_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(H_FILES
  #include/alliance/alliance.h
)

set(CPP_FILES
  #src/alliance/alliance.cpp

)

add_executable(${PROJECT_NAME}
  src/alliance_novo.cpp
  ${CPP_FILES}
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

catkin_add_gtest(${PROJECT_NAME}_utest
  test/utest.cpp
  ${CPP_FILES}
)

add_dependencies(${PROJECT_NAME}_utest
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_utest
  ${catkin_LIBRARIES}
)
