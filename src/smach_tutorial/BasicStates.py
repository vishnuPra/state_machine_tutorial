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
        rospy.sleep(2.0)
        return "continue"
    
##-----------------------------------------------------------------------------------
##Exercise 1
        
#class WaitState(smach.State):
    

    
##-----------------------------------------------------------------------------------
##Exercise 2
        
class MessageReader(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=[""], input_keys=[], output_keys=[], io_keys=[])
    
    def execute(self, ud):
        return ""

    
##-----------------------------------------------------------------------------------
##Exercise 3
        
class MessageReader2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=[""], input_keys=[], output_keys=[], io_keys=[])
    
    def execute(self, ud):
        return ""



##-----------------------------------------------------------------------------------

## Set State for exercice 2 and 3----------------------

class Set(smach.State):
    #this state write inside the userdata "msg", the message set as parameter.
    def __init__(self, msg = ""):
       smach.State.__init__(self, outcomes=["done"], input_keys=["Set_msg_in"],output_keys=["Set_msg_out"])
       self.msg = msg
    
    def execute(self, ud):
        rospy.loginfo("Current userdata 'msg' : %s" %(ud.Set_msg_in)) #print in the rospy log at level info
        ud.Set_msg_out = self.msg
        rospy.sleep(3.0)
        return "done"
## -----------------------------------------------------------------------------------------

## Already defined state machine. 
## DON'T DO ANY CHANGE BELOW !!!!

def WaitSM():
    Wait_sm = smach.StateMachine(outcomes=["exit"])
    
    with Wait_sm:
        Wait_sm.add('Wait1', EmptyState(), transitions={"continue": 'Wait2'})
        Wait_sm.add('Wait2', EmptyState(), transitions={"continue": 'Wait3'})
        Wait_sm.add('Wait3', EmptyState(), transitions={"continue": "exit"})
    
    return Wait_sm
##--------------------------

def WaitSM2():
    Wait_sm = smach.StateMachine(outcomes=["exit"])
    Wait_sm.userdata.sleep_time = 2.0 
    
    rospy.logwarn("Sleep time set to 2.0")
    with Wait_sm:
        Wait_sm.add('Wait1', WaitState(), transitions={"continue": 'Wait2'})
        Wait_sm.add('Wait2', WaitState(), transitions={"continue": 'Wait3'})
        Wait_sm.add('Wait3', WaitState(), transitions={"continue": "exit"})
    
    return Wait_sm

def SetPrintStateMachine():
    SetPrint_sm = smach.StateMachine(outcomes=["exit"])
    SetPrint_sm.userdata.msg = "This is the initial data"
    
    with SetPrint_sm:
        SetPrint_sm.add('Set', Set("Hello World"), transitions={"done": 'Print'},
                                                 remapping={"Set_msg_in":"msg",
                                                 "Set_msg_out":"msg"})
        SetPrint_sm.add('Print', MessageReader(), transitions={"continue" : 'Print_again'},
                                         remapping={"msg":"msg"})
        
        SetPrint_sm.add('Print_again', MessageReader(), transitions={"continue" : "exit"},
                                         remapping={"msg":"msg"})
    
    return SetPrint_sm


def SetPrintStateMachine2():
    SetPrint_sm = smach.StateMachine(outcomes=["exit"])
    SetPrint_sm.userdata.msg = "This is the initial data"
    SetPrint_sm.userdata.reset = True
    
    with SetPrint_sm:
        SetPrint_sm.add('Set', Set("Hello World"), transitions={"done": 'Print'},
                                                 remapping={"Set_msg_in":"msg",
                                                 "Set_msg_out":"msg"})
        SetPrint_sm.add('Print', MessageReader2(), transitions={"continue" : 'Print_again',
                                                                "empty" : "exit"},
                                         remapping={"msg":"msg"})
        
        SetPrint_sm.add('Print_again', MessageReader2(), transitions={"continue" : "exit",
                                                                "empty" : "exit"},
                                         remapping={"msg":"msg"})
    
    return SetPrint_sm
##--------------------------
## main functions

def main():
    
    Wait_sm = WaitSM()
    
    introspection_server = smach_ros.IntrospectionServer('SM', Wait_sm, '/SM_root')
    introspection_server.start()
    rospy.sleep(3.0)
    outcome = Wait_sm.execute()
    rospy.loginfo("Result : " + outcome)
    introspection_server.stop()
    
def main1():
    
    Wait_sm = WaitSM2()
    
    introspection_server = smach_ros.IntrospectionServer('SM', Wait_sm, '/SM_root')
    introspection_server.start()
    rospy.sleep(3.0)
    outcome = Wait_sm.execute()
    rospy.loginfo("Result : " + outcome)
    introspection_server.stop()
    
def main2():
    
    SimpleSM = SetPrintStateMachine()
    
    introspection_server = smach_ros.IntrospectionServer('SM', SimpleSM, '/SM_root')
    introspection_server.start()
    rospy.sleep(3.0)
    outcome = SimpleSM.execute()
    rospy.loginfo("Result : " + outcome)
    introspection_server.stop()
    
def main3():
    
    
    SimpleSM = SetPrintStateMachine2()
    
    introspection_server = smach_ros.IntrospectionServer('SM', SimpleSM, '/SM_root')
    introspection_server.start()
    rospy.sleep(3.0)
    outcome = SimpleSM.execute()
    rospy.loginfo("Result : " + outcome)
    introspection_server.stop()
    
##-----------------------------------------------------------------------------------
if __name__ == '__main__':
    rospy.init_node('tutorial_node')
    exercise = rospy.get_param('tutorial_node/exercise',0) 
    if(exercise == 0):
        main()
    elif(exercise == 1):
        main1()
    elif(exercise == 2):
        main2()
    elif(exercise == 3):
        main3()
    else:
        rospy.logerr("Exercise not listed")