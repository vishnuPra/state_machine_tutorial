#!/usr/bin/env python

import rospy
import smach
import smach_ros

class Set(smach.State):
    #this state write inside the userdata "msg", the message set as parameter.
    def __init__(self, msg = ""):
       smach.State.__init__(self, outcomes=["done"], input_keys=["Set_msg_in"],output_keys=["Set_msg_out"])
       self.msg = msg
    
    def execute(self, ud):
        rospy.loginfo("Current userdata 'msg' : %s" %(ud.Set_msg_in)) #print in the rospy log at level info
        ud.Set_msg_out = self.msg
        rospy.sleep(2.0)
        return "done"

class MessageReader(smach.State):
    #this state write inside the userdata "msg", the message set as parameter.
    def __init__(self, msg = ""):
       smach.State.__init__(self, outcomes=["done"], input_keys=["msg"])
       self.msg = msg
    
    def execute(self, ud):
        rospy.loginfo("Current userdata 'msg' : %s" %(ud.msg)) #print in the rospy log at level info
        rospy.sleep(2.0)
        return "done"    


def SetPrintStateMachine():
    SetPrint_sm = smach.StateMachine(outcomes=["exit"])
    SetPrint_sm.userdata.msg = "Message in user data"
    
    with SetPrint_sm:
        SetPrint_sm.add('Set', Set("Hello World"), transitions={"done": 'Print'},
                                                 remapping={"Set_msg_in":"msg",
                                                 "Set_msg_out":"msg"})
        SetPrint_sm.add('Print', MessageReader(), transitions={"done" : 'exit'},
                                         remapping={"msg":"msg"})
    
    return SetPrint_sm
##-----------------------------------------------------------------------------------
##Exercice 1
##define a state machine that go 3 time into the Foo state then leave

class Foo(smach.State):
    #each time that this state is executed it increased his inside counter by one
    def __init__(self):
       smach.State.__init__(self, outcomes=["continue", "out"])
       self.counter = 0
    
    def execute(self, ud):
        self.counter = self.counter + 1
        rospy.loginfo("Current Counter : %d"%(self.counter))
        rospy.sleep(2)
        if(self.counter > 2):
            return "out"
        else:
            return "continue"
            
class Bar(smach.State):
    def __init__(self):
       smach.State.__init__(self, outcomes=["continue"])
       self.counter = 0
    
    def execute(self, ud):
        rospy.sleep(2)
        return "continue"

##-----------------------------------------------------------------------------------

def FooBarStateMachine():
    FooBar_sm = smach.StateMachine(outcomes=["exit"])
    
    with FooBar_sm:
        FooBar_sm.add('Foo', Foo(), transitions={"continue": 'Bar',
                                                 "out": "exit"})
        FooBar_sm.add('Bar', Bar(), transitions={"continue" : 'Foo'})
        
    return FooBar_sm
    
    return SetPrint_sm


##-----------------------------------------------------------------------------------    
##Exercice 2
##define a state machine with the SetPrintSM nested

class EmptyState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["continue"], input_keys=[], output_keys=[], io_keys=[])
    
    def execute(self, ud):
        rospy.sleep(2.0)
        return "continue" 
   
def NestedStateMachine():
    main_sm = smach.StateMachine(outcomes=["over"])
    
    with main_sm:
        main_sm.add('Wait',EmptyState(),transitions={"continue":'SetPrintSM'})
        main_sm.add('SetPrintSM',SetPrintStateMachine(),transitions={"exit":"over"})
    
    return main_sm


