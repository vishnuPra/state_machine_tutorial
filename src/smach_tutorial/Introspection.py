#!/usr/bin/env python

import smach
import rospy
import smach_ros

import state_machine_tutorial
from smach_tutorial.BasicStateMachine import *
##-----------------------------------------------------------------------------------

def main():
    
    SimpleSM = SetPrintStateMachine()
    
    introspection_server = smach_ros.IntrospectionServer('SM', SimpleSM, '/SM_root')
    introspection_server.start()
    
    outcome = SimpleSM.execute()
    rospy.loginfo("Result : " + outcome)
    introspection_server.stop()
    
def main1():
#Create a main function that launch the FooBarStateMachine()

    
##-----------------------------------------------------------------------------------

if __name__ == '__main__':
    rospy.init_node('tutorial_node')
    main()
            

        
