#!/usr/bin/env python

import rospy
import smach
import tf
import actionlib
import smach_ros

from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult
##-----------------------------------------------------------------------------------
##Example 1

class SetGoal(smach.State):
    #this state write inside the userdata "msg", the message set as parameter.
    def __init__(self):
        smach.State.__init__(self, outcomes=["done","invalid"], input_keys=["goal_in"], output_keys=["goal_out"])

    def execute(self, ud):
        if(isinstance(ud.goal_in,list)):
           if(len(ud.goal_in) == 3):
               # set goal
               goal = MoveBaseGoal()
               goal.target_pose.header.stamp = rospy.Time.now()
               goal.target_pose.header.frame_id = '/map'
               goal.target_pose.pose.position.x = ud.goal_in[0]
               goal.target_pose.pose.position.y = ud.goal_in[1]
               goal.target_pose.pose.position.z = 0.0
               #convert from euler to quaternion
               quat = tf.transformations.quaternion_from_euler(0.0, 0.0, ud.goal_in[2], 'rzyx')
               #set the orientation
               goal.target_pose.pose.orientation.w = quat[0]
               goal.target_pose.pose.orientation.x = quat[1]
               goal.target_pose.pose.orientation.y = quat[2]
               goal.target_pose.pose.orientation.z = quat[3]
               #set the goal in the userdata
               ud.goal_out = goal

               return "done"
           else:
               return "invalid"
        else:
            return "invalid"

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

class Wait(smach.State):
    def __init__(self, sleep_time=10):
        smach.State.__init__(self, outcomes=["done", "preempted"])
        self.sleep_time = sleep_time

    def execute(self, ud):
        #we compute the sleep time
        ros_timeout_ = rospy.Time().now() + rospy.Duration(self.sleep_time)
        while(rospy.Time.now() < ros_timeout_):
            if self.preempt_requested() or rospy.is_shutdown():
                rospy.logwarn("Preempted ! (or shutdown)")
                return "preempted"
            else:
                rospy.sleep(0.1)

        return "done"


def MovingSM():
    Moving_sm = smach.StateMachine(outcomes=["exit"])
    Moving_sm.userdata.goal = [0.592, -0.553,0.0]

    with Moving_sm:
        Moving_sm.add('SetGoal', SetGoal(), transitions={"done" : 'MoveBase',
                                                      "invalid" : "exit"},
                                          remapping={"goal_in"  : "goal",
                                                     "goal_out" : "goal"})

        Moving_sm.add('MoveBase', MoveBase_ac(), transitions={"succeeded"  : 'Wait',
                                                             "aborted"   : "exit",
                                                             "preempted" : "exit"},
                                                  remapping={"goal"      :"goal"})

        Moving_sm.add('Wait',  Wait(1.0), transitions={"done"      : "exit",
                                                      "preempted" : "exit"})

    return Moving_sm

##-----------------------------------------------------------------------------------

##this state will set the amcl init pose so we don't get lost at first
class SetInitialPose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["position_set"])
        self.init_pose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)

    def execute(self, ud):
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
        return "position_set"


##this state read a list and extract the next point
class NextGoal(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["next_point", "finished"], io_keys=["goal_list","next_goal"])
        self.current_pt_index = 0

    def execute(self, ud):
        ##check if we have finished the list
        if(self.current_pt_index >= len(ud.goal_list)):
            rospy.loginfo("Trajectory finished !")
            return "finished"
        else:
            #Get the current_pt from the list
            ud.next_goal = ud.goal_list[self.current_pt_index]
            self.current_pt_index = self.current_pt_index +1
            rospy.loginfo("Going to next point : %s"%str(ud.next_goal))
            return "next_point"


def MovingSMNested():
    Moving_sm = smach.StateMachine(outcomes=["next","aborted"],input_keys=["goal"])


    with Moving_sm:
        Moving_sm.add('SetGoal', SetGoal(), transitions={"done" : 'MoveBase',
                                                      "invalid" : "aborted"},
                                          remapping={"goal_in"  : "goal",
                                                     "goal_out" : "goal"})

        Moving_sm.add('MoveBase', MoveBase_ac(), transitions={"succeeded"  : 'Wait',
                                                             "aborted"   : "aborted",
                                                             "preempted" : "aborted"},
                                                  remapping={"goal"      :"goal"})

        Moving_sm.add('Wait',  Wait(2.0), transitions={"done"      : "next",
                                                      "preempted" : "aborted"})

    return Moving_sm

def FullTrajectorySM():
    FullTrajectory_sm = smach.StateMachine(outcomes=["aborted", "finished"])
    #the goal list
    FullTrajectory_sm.userdata.goal_list = [[0.592, -0.553,0.0],
                                            [1.862, 0.546,-3.14],
                                            [-1.605, 1.387,-1.57],
                                            [-1.786, -0.453,0.0]]

    with FullTrajectory_sm:
        FullTrajectory_sm.add('Init', SetInitialPose(), transitions={"position_set":'NextGoal'})

        FullTrajectory_sm.add('NextGoal', NextGoal(), transitions={"next_point":'Moving',
                                                                   "finished" : "finished"},
                                                      remapping={"goal_list" : "goal_list",
                                                                 "next_goal" : "goal"})

        FullTrajectory_sm.add('Moving', MovingSMNested(), transitions={"next" : 'NextGoal',
                                                                "aborted" : "aborted"},
                                                     remapping={"goal" :  "goal"})

    return FullTrajectory_sm

def main():

    Trajectory_sm = MovingSM()

    introspection_server = smach_ros.IntrospectionServer('SM', Trajectory_sm, '/SM_root')
    introspection_server.start()

    Trajectory_sm.execute()

    introspection_server.stop()


def main1():

    FullTrajectory_sm = FullTrajectorySM()

    introspection_server = smach_ros.IntrospectionServer('SM', FullTrajectory_sm, '/SM_root')
    introspection_server.start()

    FullTrajectory_sm.execute()

    introspection_server.stop()


if __name__ == '__main__':
    rospy.init_node('tutorial_node')
    main1()
