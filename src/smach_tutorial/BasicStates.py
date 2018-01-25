#!/usr/bin/env python

import smach
import rospy
import smach_ros

##-----------------------------------------------------------------------------------
##Exercise 0

class EmptyState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["continue"], input_keys=[], output_keys=[], io_keys=[])
    
    def execute(self, ud):
        rospy.sleep(1)
        return "continue"
    
##-----------------------------------------------------------------------------------
##Exercise 1
        
#class WaitState(smach.State):
    

    
##-----------------------------------------------------------------------------------
##Exercise 2
        
#class MessageReader(smach.State):

    
##-----------------------------------------------------------------------------------
##Exercise 3
        
#class MessageReader2(smach.State):

            

        
