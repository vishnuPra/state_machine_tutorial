cmake_minimum_required(VERSION 2.8.3)
project(state_machine_tutorial)

find_package(catkin REQUIRED COMPONENTS
  airbus_ssm_core
  airbus_cobot_gui
  airbus_ssm_plugin
  rospy
  smach
  smach_viewer

)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS   
  airbus_ssm_core
  airbus_cobot_gui
  airbus_ssm_plugin
  rospy
  smach
  smach_viewer
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
)

