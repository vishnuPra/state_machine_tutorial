cmake_minimum_required(VERSION 2.8.3)
project(state_machine_tutorial)

find_package(catkin REQUIRED COMPONENTS

)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS   
  rospy
  smach
  smach_viewer
  )

