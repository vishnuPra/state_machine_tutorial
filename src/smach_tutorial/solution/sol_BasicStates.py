#!/usr/bin/env python

import smach
import rospy
import qt_smach_viewer.introspection


##-----------------------------------------------------------------------------------
##Exercise 1

class WaitState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["continue"], input_keys=["sleep_time"])

    def execute(self, ud):
        rospy.sleep(ud.sleep_time)
        return "continue"

##-----------------------------------------------------------------------------------
##Exercise 2

class MessageReader(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["continue"], io_keys=["msg"])

    def execute(self, ud):
        rospy.loginfo("Message in the userdata : " +ud.msg) #print in the rospy log at level info
        #print(ud.msg)
        ud.msg = ''#reset the message
        rospy.sleep(1.0)
        return "continue"

##-----------------------------------------------------------------------------------
##Exercise 3

class MessageReader2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["continue","empty"], io_keys=["msg","reset"])

    def execute(self, ud):
        rospy.sleep(1.0)
        if(ud.msg <> ''):
            rospy.loginfo(ud.msg) #print in the rospy log at level info
            #print(ud.msg)
            if ud.reset:
                rospy.logwarn("Emptying message !")
                ud.msg = ''#reset the message
            return "continue"
        else:
            rospy.logerr("Message Empty !")
            return "empty"
