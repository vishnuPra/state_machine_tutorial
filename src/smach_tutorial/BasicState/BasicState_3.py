#!/usr/bin/env python

import smach
import rospy
import smach_ros


# Exercise 3: Read a message from a userdata and overwrite it, if needed
# Fill this class
class MessageReader2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=[""], input_keys=[], output_keys=[], io_keys=[])

    def execute(self, ud):
        return ""


class Set(smach.State):
    #this state write inside the userdata "msg", the message set as parameter.
    def __init__(self, msg= ""):
       smach.State.__init__(self, outcomes=["done"], input_keys=["Set_msg_in"],output_keys=["Set_msg_out"])
       self.msg = msg

    def execute(self, ud):
        rospy.loginfo("Current userdata 'msg' : %s" %(ud.Set_msg_in)) #print in the rospy log at level info
        ud.Set_msg_out = self.msg
        rospy.sleep(3.0)
        return "done"


def SetPrintStateMachine2():
    SetPrint_sm = smach.StateMachine(outcomes=["exit"])
    SetPrint_sm.userdata.msg = "This is the initial data"
    SetPrint_sm.userdata.reset = True

    with SetPrint_sm:
        SetPrint_sm.add('Set', Set("Hello World"), transitions={"done": 'Print'},
                        remapping={"Set_msg_in": "msg", "Set_msg_out": "msg"})
        SetPrint_sm.add('Print', MessageReader2(), transitions={"continue": 'Print_again',
                                                                "empty": "exit"})

        SetPrint_sm.add('Print_again', MessageReader2(), transitions={"continue": "exit",
                                                                "empty": "exit"})
    return SetPrint_sm


def main():
    SimpleSM = SetPrintStateMachine2()

    introspection_server = smach_ros.IntrospectionServer('SM', SimpleSM, '/SM_root')
    introspection_server.start()
    rospy.sleep(3.0)
    outcome = SimpleSM.execute()
    rospy.loginfo("Result : " + outcome)
    introspection_server.stop()


if __name__ == '__main__':
    rospy.init_node('tutorial_node')
    main()
