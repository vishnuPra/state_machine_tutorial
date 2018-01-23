#!/usr/bin/env python


import rospy
import ast
import tf
import actionlib
import smach_ros

from airbus_ssm_core import ssm_state

from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult


class FooState(ssm_state.ssmState):
    #each time that this state is executed it increased his inside counter by one
    def __init__(self):
       ssm_state.ssmState.__init__(self, outcomes=["continue", "out"])
       self.counter = 0
    
    def execution(self, ud):
        self.counter = self.counter + 1
        rospy.loginfo("Current Counter : %d"%(self.counter))
        rospy.sleep(2)
        if(self.counter > 2):
            return "out"
        else:
            return "continue"
            
class BarState(ssm_state.ssmState):
    def __init__(self):
       ssm_state.ssmState.__init__(self, outcomes=["continue"])
       self.counter = 0
    
    def execution(self, ud):
        rospy.sleep(2)
        return "continue"

class EmptyState(ssm_state.ssmState):
    def __init__(self):
        ssm_state.ssmState.__init__(self, outcomes=["next"], io_keys=[])
    
    def execution(self, ud):
        rospy.sleep(1)
        return "next"
        
class WaitState(ssm_state.ssmState):
    def __init__(self):
        ssm_state.ssmState.__init__(self, outcomes=["continue"], io_keys=["sleep_time"])
        
    def execution(self, ud):
        rospy.sleep(sleep_time)
        return "continue"
        
class MessageReader(ssm_state.ssmState):
    def __init__(self):
        ssm_state.ssmState.__init__(self, outcomes=["continue"], io_keys=["msg"])
        
    def execution(self, ud):
        rospy.loginfo(ud.msg) #print in the rospy log at level info
        #print(ud.msg)
        ud.msg = ''#reset the message
        return "continue"
        
class MessageReader2(ssm_state.ssmState):
    def __init__(self):
        ssm_state.ssmState.__init__(self, outcomes=["continue","empty"], io_keys=["msg"])
        
    def execution(self, ud):
        if(msg <> ''):
            rospy.loginfo(ud.msg) #print in the rospy log at level info
            #print(ud.msg)
            ud.msg = ''#reset the message
            return "continue"
        else:
            rospy.logerror("Message Empty !")
            return "empty"
        

        
class SetGoal(ssm_state.ssmState):
    #this state write inside the userdata "msg", the message set as parameter.
    def __init__(self):
        ssm_state.ssmState.__init__(self, outcomes=["done","invalid"], io_keys=["goal"]) #remapping from the goal data from scxml
    
    def execution(self, ud):
        print(ud.goal)
        if(isinstance(ud.goal,basestring)):
            ud.goal = ast.literal_eval(ud.goal) #convert the string into a list 
        
        if(isinstance(ud.goal,list)): #change from goal_in
           if(len(ud.goal) == 3): #change from goal_in
               # set goal
               goal = MoveBaseGoal()
               goal.target_pose.header.stamp = rospy.Time.now()
               goal.target_pose.header.frame_id = '/map'
               goal.target_pose.pose.position.x = ud.goal[0] #change from goal_in 
               goal.target_pose.pose.position.y = ud.goal[1] #change from goal_in
               goal.target_pose.pose.position.z = 0.0
               #convert from euler to quaternion
               quat = tf.transformations.quaternion_from_euler(0.0, 0.0, ud.goal[2], 'rzyx') #change from goal
               #set the orientation
               goal.target_pose.pose.orientation.w = quat[0]
               goal.target_pose.pose.orientation.x = quat[1]
               goal.target_pose.pose.orientation.y = quat[2]
               goal.target_pose.pose.orientation.z = quat[3]
               #set the goal in the userdata
               ud.goal = goal 
               
               return "done"
           else:
               return "invalid"
        else:
            return "invalid"
        
class Wait_ssm(ssm_state.ssmState):
    def __init__(self):
        ssm_state.ssmState.__init__(self, outcomes=["done"], io_keys=["sleep_time"])
        
    def execution(self, ud):
        #we compute the sleep time
        if(isinstance(ud.sleep_time,basestring)):
            ud.sleep_time = ast.literal_eval(ud.sleep_time) #convert the string into a int 
        ros_timeout_ = rospy.Time().now() + rospy.Duration(ud.sleep_time) #come from the data model
        while(rospy.Time.now() < ros_timeout_):
            if self.preempt_requested() or rospy.is_shutdown():
                rospy.logwarn("Preempted ! (or shutdown)")
                return "preempt"
            else:
                rospy.sleep(0.1)
                
        return "done"
    
class MoveBase_ac(ssm_state.ssmState):
    #this state connect to the action client and make the robot move
    def __init__(self):
        ssm_state.ssmState.__init__(self, outcomes=["succeeded","aborted"], io_keys=["goal"]) #change from input_keys
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
                
                
    def execution(self, ud):
        
        connection = self.wait_for_server()
        if(connection == "preempt"):
            return "preempt"
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
            return "preempt"
        elif result == "timeout":
            return "aborted"
        else:
            pass
        
        return "succeeded"
    
##this state will set the amcl init pose so we don't get lost at first
class SetInitialPose(ssm_state.ssmState):
    def __init__(self):
        ssm_state.ssmState.__init__(self, outcomes=["position_set"])
        self.init_pose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)
    
    def execution(self, ud):
        rospy.sleep(1)
        #filling the amcl message
        init_pose = PoseWithCovarianceStamped()
        init_pose.header.stamp = rospy.Time.now()
        init_pose.header.frame_id = "map"
        init_pose.pose.pose.position.x = -1.93233079003
        init_pose.pose.pose.position.y = -0.575442970722
        ##other are set to 0
        init_pose.pose.pose.orientation.z = 0.0841459150786
        init_pose.pose.pose.orientation.w = 0.996453443456
        init_pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 
                                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
        
        self.init_pose_pub.publish(init_pose)
        rospy.sleep(1)
        return "position_set"

        
##this state read a list and extract the next point
class NextGoal(ssm_state.ssmState):
    def __init__(self):
        ssm_state.ssmState.__init__(self, outcomes=["next_point", "finished"], io_keys=["goal_list","goal"])
        self.current_pt_index = 0
        
    def execution(self, ud):
        ##check if we have finished the list
        if(isinstance(ud.goal_list,basestring)):
            ud.goal_list = ast.literal_eval(ud.goal_list) #convert the string into a list 
        if(self.current_pt_index >= len(ud.goal_list)):
            rospy.loginfo("Trajectory finished !")
            return "finished"
        else:
            #Get the current_pt from the list
            ud.goal = ud.goal_list[self.current_pt_index]
            self.current_pt_index = self.current_pt_index +1
            rospy.loginfo("Going to next point : %s"%str(ud.goal))
            return "next_point"
