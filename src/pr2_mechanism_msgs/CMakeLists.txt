cmake_minimum_required(VERSION 2.8.3)
project(pr2_mechanism_msgs)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
  actionlib_msgs
  actionlib
)

add_message_files(
  FILES
  ActuatorStatistics.msg
  ControllerStatistics.msg
  JointStatistics.msg
  MechanismStatistics.msg
)

add_service_files(
  FILES
  ListControllers.srv
  ListControllerTypes.srv
  LoadController.srv
  ReloadControllerLibraries.srv
  SwitchController.srv
  UnloadController.srv
)

add_action_files(
  FILES
  SwitchController.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)



