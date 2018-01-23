#!/usr/bin/env python

import rospy
import smach
import smach_ros

from std_msgs.msg import String, Bool, Empty

#---------------------------------------------------------------------------------------------------------------
        
class WaitForMessage(smach.State):
    def __init__(self, timeout=10):
        smach.State.__init__(self, outcomes=["timeout", "message_received","preempted"], output_keys=["msg"])
        self.sub = rospy.Subscriber("/message",String, self.message_received_cb)
        self.msg_received = False
        self.msg = None
        self.timeout = timeout
    
    def message_received_cb(self, msg):
        self.msg = msg.data
        self.msg_received = True
        
    def execute(self, ud):
        #set the value to false at the begin of execution because I want it to wait eveytime I call execute the state
        self.msg_received = False
        self.msg = None
        ros_timeout_ = rospy.Time().now() + rospy.Duration(self.timeout)
        while(self.msg_received == False):
            if rospy.Time.now() > ros_timeout_:
                rospy.logwarn("Timed Out")
                return "timeout"
            elif self.preempt_requested() or rospy.is_shutdown():
                rospy.logwarn("Preempted ! (or shutdown)")
                return "preempted"
            else:
                rospy.sleep(0.1)
        rospy.loginfo("Message received : '%s'" %self.msg)
        ud.msg = self.msg
        return "message_received"
    
#---------------------------------------------------------------------------------------------------------------    
    
class MoveBase_ac(smach.State):
    #this state connect to the action client and make the robot move
    def __init__(self):
       smach.State.__init__(self, outcomes=["succeeded","aborted","preempted"], input_keys=["goal"])
       self.action_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction) 
       self._tosv = 10.0 #timeout server
       self._toaction = 60.0 #timeout action
       self._done = False #execution check
       
    def wait_for_server(self): #wait for the server but is able to be preempted if needed
        to_ = rospy.Time().now() + rospy.Duration(self._tosv)
        server_connected = False
        while server_connected == False:
            rospy.sleep(0.01)
            if self.preempt_requested() or rospy.is_shutdown():
                self.service_preempt()
                return "preempt"
            if rospy.Time.now() > to_:
                rospy.logwarn("'%s connection to server timed out : it will return the following outcome '%s'"
                             %(self.__class__.__name__,"aborted"))
                return "timeout"
            else:
                server_connected = self.action_client.wait_for_server(rospy.Duration(0.1))
                
    def done_cb(self, status, result):
        #is call when the goal is reached
        self._done = True
                
    def wait_for_result(self): #wait for the action result but is able to be preempted if needed
        to_ = rospy.Time().now() + rospy.Duration(self._toaction)
        while self._done == False:
            if self.preempt_requested() or rospy.is_shutdown():
                self.service_preempt()
                self.action_client.cancel_goal()
                return "preempt"
            if rospy.Time.now() > to_:
                rospy.logwarn("'%s action timed out : it will return the following outcome '%s'"
                             %(self.__class__.__name__,"aborted"))
                self.action_client.cancel_goal()
                return "timeout"
            else:
                rospy.sleep(0.01)
                
                
    def execute(self, ud):
        
        connection = self.wait_for_server()
        if(connection == "preempt"):
            return "preempted"
        elif(connection == "timeout"):
            return "aborted"
        else:
            #Connected
            pass
        
        self._done = False
        self.action_client.send_goal(ud.goal, done_cb = self.done_cb, active_cb=None, feedback_cb = None)
        result = self.wait_for_result()
        if result == "preempt":
            rospy.loginfo("goal preempted")
            return "preempted"
        elif result == "timeout":
            return "aborted"
        else:
            pass
        
        return "succeeded"
    



        
