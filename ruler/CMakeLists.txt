cmake_minimum_required(VERSION 2.8.3)

project(ruler)

find_package(Boost REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  pluginlib
  roscpp
  ruler_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    pluginlib
    roscpp
    ruler_msgs
  DEPENDS Boost
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

set(H_FILES
  include/nodes/ruler_node.h
  include/nodes/metrics_service_server.h
  include/ruler/plugins/metrics_calculator.h
  include/ruler/consumable_resource.h
  include/ruler/consumable_resource_reservation_request.h
  include/ruler/continuous_consumable_resource.h
  include/ruler/continuous_reusable_resource.h
  include/ruler/discrete_consumable_resource.h
  include/ruler/discrete_reusable_resource.h
  include/ruler/event_types.h
  include/ruler/metrics_estimator.h
  include/ruler/profile.h
  include/ruler/resource.h
  include/ruler/resource_interface.h
  include/ruler/resource_reservation_request.h
  include/ruler/reusable_resource.h
  include/ruler/reusable_resource_reservation_request.h
  include/ruler/robot.h
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
  include/utilities/event.h
  include/utilities/functions/buffered_function.h
  include/utilities/functions/continuous_exponential_function.h
  include/utilities/functions/continuous_linear_function.h
  include/utilities/functions/continuous_pulse_function.h
  include/utilities/functions/continuous_sample_holder.h
  include/utilities/functions/continuous_step_function.h
  include/utilities/functions/discrete_exponential_function.h
  include/utilities/functions/discrete_linear_function.h
  include/utilities/functions/discrete_pulse_function.h
  include/utilities/functions/discrete_step_function.h
  include/utilities/functions/discrete_sample_holder.h
  include/utilities/functions/exponential_function.h
  include/utilities/functions/function.h
  include/utilities/functions/functions.h
  include/utilities/functions/linear_function.h
  include/utilities/functions/probability_density_function.h
  include/utilities/functions/pulse_function.h
  include/utilities/functions/sample_holder.h
  include/utilities/functions/step_function.h
  include/utilities/functions/time_probability_density_function.h
  include/utilities/functions/unary_pulse_function.h
  include/utilities/functions/unary_step_function.h
  include/utilities/functions/unary_sample_holder.h
  include/utilities/functions/value_change_event.h
  include/utilities/has_id.h
  include/utilities/has_name.h
  include/utilities/interval.h
  include/utilities/observer.h
  include/utilities/ros_message_converter.h
  include/utilities/ros_node.h
  include/utilities/ros_service_server.h
  include/utilities/signal_type.h
  include/utilities/signal_types.h
  include/utilities/subject.h
  include/utilities/toggle_event.h
  include/utilities/unary_signal_type.h
  include/utilities/utilities.h
)

set(CPP_FILES
  src/nodes/ruler_node.cpp
  src/nodes/metrics_service_server.cpp
  src/ruler/continuous_consumable_resource.cpp
  src/ruler/continuous_reusable_resource.cpp
  src/ruler/discrete_consumable_resource.cpp
  src/ruler/discrete_reusable_resource.cpp
  src/ruler/event_types.cpp
  src/ruler/metrics_estimator.cpp
  src/ruler/resource_interface.cpp
  src/ruler/resource_reservation_request.cpp
  src/ruler/robot.cpp
  src/ruler/task.cpp
  src/ruler/task_event.cpp
  src/ruler/unary_consumable_resource.cpp
  src/ruler/unary_reusable_resource.cpp
  src/utilities/continuous_signal_type.cpp
  src/utilities/discrete_signal_type.cpp
  src/utilities/exception.cpp
  src/utilities/event.cpp
  src/utilities/functions/continuous_exponential_function.cpp
  src/utilities/functions/continuous_linear_function.cpp
  src/utilities/functions/continuous_pulse_function.cpp
  src/utilities/functions/continuous_sample_holder.cpp
  src/utilities/functions/continuous_step_function.cpp
  src/utilities/functions/discrete_exponential_function.cpp
  src/utilities/functions/discrete_linear_function.cpp
  src/utilities/functions/discrete_sample_holder.cpp
  src/utilities/functions/probability_density_function.cpp
  src/utilities/functions/discrete_pulse_function.cpp
  src/utilities/functions/discrete_step_function.cpp
  src/utilities/functions/time_probability_density_function.cpp
  src/utilities/functions/unary_pulse_function.cpp
  src/utilities/functions/unary_step_function.cpp
  src/utilities/functions/unary_sample_holder.cpp
  src/utilities/has_name.cpp
  src/utilities/observer.cpp
  src/utilities/ros_node.cpp
  src/utilities/signal_types.cpp
  src/utilities/subject.cpp
  src/utilities/toggle_event.cpp
  src/utilities/unary_signal_type.cpp
)

add_library(${PROJECT_NAME}
  ${CPP_FILES}
)

add_dependencies(${PROJECT_NAME}
  ${Boost_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node
  src/ruler_node.cpp
  ${CPP_FILES}
)

add_dependencies(${PROJECT_NAME}_node
  ${Boost_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_node
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

catkin_add_gtest(${PROJECT_NAME}_utest
  test/utest.cpp
  ${CPP_FILES}
)

add_dependencies(${PROJECT_NAME}_utest
  ${Boost_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_utest
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)
