#!/usr/bin/env python

import rospy
import smach
import smach_ros

from std_msgs.msg import String, Bool, Empty

#---------------------------------------------------------------------------------------------------------------

class WaitForMessage(smach.State):
    def __init__(self, timeout=10):
        smach.State.__init__(self, outcomes=["timeout", "message_received","preempted"], output_keys=["msg"]) #all outcomes and user data are define
        self.sub = rospy.Subscriber("/message",String, self.message_received_cb)#initialise the subscriber
        self.msg_received = False #Boolean that act like a flag when we receive a message
        self.msg = None #Where we will put the content of the message
        self.timeout = timeout #The timeout float

    def message_received_cb(self, msg): #callback of the subscriber
        self.msg = msg.data #we copy the content of the message inside out place holder
        self.msg_received = True #set the flag to true

    def execute(self, ud):
        #set the value to false at the begin of execution because I want it to wait eveytime I call execute the state
        self.msg_received = False #
        self.msg = None #empty the place holder
        ros_timeout_ = rospy.Time().now() + rospy.Duration(self.timeout) #compute the timeout rostime
        while(self.msg_received == False): #until the flag in up
            if rospy.Time.now() > ros_timeout_: #check if we are over the timeout rostime
                rospy.logwarn("Timed Out") #Warning message
                return "timeout" #timeout
            elif self.preempt_requested() or rospy.is_shutdown(): #check if we are preempted or if ctrl+c
                rospy.logwarn("Preempted ! (or shutdown)") #waning message
                return "preempted" #preempted
            else:
                rospy.sleep(0.1) #free the thread for callback checking
        rospy.loginfo("Message received : '%s'" %self.msg) #information message
        ud.msg = self.msg #copy the message content inside the userdata
        return "message_received" #message received
