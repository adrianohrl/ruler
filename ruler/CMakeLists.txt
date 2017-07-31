cmake_minimum_required(VERSION 2.8.3)

project(ruler)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  ruler_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    ruler_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(H_FILES
  include/ruler/consumable_resource.h
  include/ruler/continuous_consumable_resource.h
  include/ruler/continuous_reusable_resource.h
  include/ruler/discrete_consumable_resource.h
  include/ruler/discrete_reusable_resource.h
  include/ruler/event.h
  include/ruler/event_types.h
  include/ruler/profile.h
  include/ruler/resource.h
  include/ruler/reusable_resource.h
  include/ruler/ruler.h
  include/ruler/task.h
  include/ruler/task_event.h
  include/ruler/task_function.h
  include/ruler/unary_consumable_resource.h
  include/ruler/unary_reusable_resource.h
  include/utilities/continuous_signal_type.h
  include/utilities/discrete_signal_type.h
  include/utilities/enum_converter.h
  include/utilities/exception.h
  include/utilities/functions/continuous_exponential_function.h
  include/utilities/functions/continuous_linear_function.h
  include/utilities/functions/continuous_pulse_function.h
  include/utilities/functions/continuous_step_function.h
  include/utilities/functions/discrete_exponential_function.h
  include/utilities/functions/discrete_linear_function.h
  include/utilities/functions/discrete_pulse_function.h
  include/utilities/functions/discrete_step_function.h
  include/utilities/functions/exponential_function.h
  include/utilities/functions/function.h
  include/utilities/functions/functions.h
  include/utilities/functions/linear_function.h
  include/utilities/functions/pulse_function.h
  include/utilities/functions/step_function.h
  include/utilities/functions/unary_pulse_function.h
  include/utilities/functions/unary_step_function.h
  include/utilities/has_id.h
  include/utilities/interval.h
  include/utilities/non_unary_signal_type.h
  include/utilities/observer.h
  include/utilities/ros_node.h
  include/utilities/signal_type.h
  include/utilities/subject.h
  include/utilities/unary_signal_type.h
  include/utilities/utilities.h
)

set(CPP_FILES
  src/ruler/continuous_consumable_resource.cpp
  src/ruler/continuous_reusable_resource.cpp
  src/ruler/discrete_consumable_resource.cpp
  src/ruler/discrete_reusable_resource.cpp
  src/ruler/event.cpp
  src/ruler/event_types.cpp
  src/ruler/task.cpp
  src/ruler/task_event.cpp
  src/ruler/unary_consumable_resource.cpp
  src/ruler/unary_reusable_resource.cpp
  src/utilities/continuous_signal_type.cpp
  src/utilities/discrete_signal_type.cpp
  src/utilities/exception.cpp
  src/utilities/functions/continuous_exponential_function.cpp
  src/utilities/functions/continuous_linear_function.cpp
  src/utilities/functions/continuous_pulse_function.cpp
  src/utilities/functions/continuous_step_function.cpp
  src/utilities/functions/discrete_exponential_function.cpp
  src/utilities/functions/discrete_linear_function.cpp
  src/utilities/functions/discrete_pulse_function.cpp
  src/utilities/functions/discrete_step_function.cpp
  src/utilities/functions/unary_pulse_function.cpp
  src/utilities/functions/unary_step_function.cpp
  src/utilities/ros_node.cpp
  src/utilities/unary_signal_type.cpp
)

add_library(${PROJECT_NAME}
  ${CPP_FILES}
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

catkin_add_gtest(utest
  test/utest.cpp
  ${CPP_FILES}
)

add_dependencies(utest
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(utest
  ${catkin_LIBRARIES}
)
