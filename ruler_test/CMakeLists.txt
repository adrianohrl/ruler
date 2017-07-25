cmake_minimum_required(VERSION 2.8.3)

project(ruler_test)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

find_package(ruler REQUIRED
  ruler_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ruler
  CATKIN_DEPENDS
    ruler_msgs
    roscpp
    std_msgs
)

include_directories(
  include
  ${ruler_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

set(H_FILES
)

set(CPP_FILES
)

add_executable(ruler_test_node src/ruler_test_node.cpp ${CPP_FILES})
add_dependencies(ruler_test_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(ruler_test_node ${catkin_LIBRARIES} ${ruler_LIBRARIES})
