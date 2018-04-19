#!/usr/bin/env python

import rospy
import smach
import smach_ros

from std_msgs.msg import String, Bool, Empty

#---------------------------------------------------------------------------------------------------------------
        
class WaitForMessage(smach.State):
    def __init__(self, timeout=10):
        smach.State.__init__(self, outcomes=["timeout", "message_received","preempted"], output_keys=["msg"])
        #HAVE FUN
        #Tips: Look at the MoveBase_ac below
        #Tips: self.sub = rospy.Subscriber("/message",String, self.message_received_cb)#initialise the subscriber
    
    def message_received_cb(self, msg):
        pass


##-----------------------------------------------------------------------------------
class EmptyState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["continue"], input_keys=[], output_keys=[], io_keys=[])
    
    def execute(self, ud):
        rospy.sleep(2.0)
        return "continue"
    
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
    


def WaitSM():
    Wait_sm = smach.StateMachine(outcomes=["timeout","message_received","preempted"])
    
    
    with Wait_sm:
        Wait_sm.add('Wait', EmptyState(), transitions={"continue" : 'WaitMsg'})
        Wait_sm.add('WaitMsg', WaitForMessage(), transitions={"timeout" : "timeout",
                                                      "message_received" : "message_received",
                                                      "preempted": "preempted"})
                                                      
    return Wait_sm

def main():
    
    Wait_sm = WaitSM()
    
    introspection_server = smach_ros.IntrospectionServer('SM', Wait_sm, '/SM_root')
    introspection_server.start()
    
    Wait_sm.execute()
    
    introspection_server.stop()
    

if __name__ == '__main__':
    rospy.init_node('tutorial_node')
    exercise = rospy.get_param('tutorial_node/exercise',0) 
    if(exercise == 0):
        main()
    else:
        rospy.logerr("Exercise not listed")


        
