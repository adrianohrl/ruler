cmake_minimum_required(VERSION 2.8.3)

project(ruler_test)

set(ROS_BUILD_TYPE Debug)

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
  include/ruler_test/ruler_test_node.h
  include/ruler_test/task_execution_simulator_node.h
  include/ruler_test/task_generator_node.h
)

set(CPP_FILES
  src/ruler_test/ruler_test_node.cpp
  src/ruler_test/task_execution_simulator_node.cpp
  src/ruler_test/task_generator_node.cpp
)

add_executable(ruler_test_node src/ruler_test_node.cpp ${CPP_FILES})

add_dependencies(ruler_test_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(ruler_test_node ${catkin_LIBRARIES} ${ruler_LIBRARIES})

add_executable(task_generator_node src/task_generator_node.cpp ${CPP_FILES})

add_dependencies(task_generator_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(task_generator_node ${catkin_LIBRARIES} ${ruler_LIBRARIES})

add_executable(task_execution_simulator_node src/task_execution_simulator_node.cpp ${CPP_FILES})

add_dependencies(task_execution_simulator_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(task_execution_simulator_node ${catkin_LIBRARIES} ${ruler_LIBRARIES})
