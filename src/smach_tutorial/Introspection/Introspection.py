#!/usr/bin/env python

import rospy
import smach_ros

from smach_tutorial.BasicStateMachine import BasicStateMachine_0,\
                                             BasicStateMachine_1,\
                                             BasicStateMachine_2
##-----------------------------------------------------------------------------------
# Example
def main():

    SimpleSM = BasicStateMachine_0.SetPrintStateMachine()

    introspection_server = smach_ros.IntrospectionServer('SM', SimpleSM, '/SM_root')
    introspection_server.start()

    outcome = SimpleSM.execute()
    rospy.loginfo("Result : " + outcome)
    introspection_server.stop()

def main1():
#Create a main function that launch the BasicStateMachine_1.FooBarStateMachine()
    pass

##-----------------------------------------------------------------------------------

if __name__ == '__main__':
    rospy.init_node('tutorial_node')
    main() #Change to main1 to call your function
