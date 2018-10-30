#!/usr/bin/env python

import rospy
import smach
import smach_ros


# Exercice 2
# define a state machine with the SetPrintStateMachine nested
class Set(smach.State):
    def __init__(self, msg=""):
        smach.State.__init__(self, outcomes=["done"], input_keys=["Set_msg_in"], output_keys=["Set_msg_out"])
        self.msg = msg

    def execute(self, ud):
        rospy.loginfo("Current userdata 'msg' : %s" %(ud.Set_msg_in))
        ud.Set_msg_out = self.msg
        rospy.sleep(2.0)
        return "done"


class MessageReader(smach.State):
    def __init__(self, msg=""):
        smach.State.__init__(self, outcomes=["done"], input_keys=["msg"])
        self.msg = msg

    def execute(self, ud):
        rospy.loginfo("Current userdata 'msg' : %s" %(ud.msg))
        rospy.sleep(2.0)
        return "done"


def SetPrintStateMachine():
    SetPrint_sm = smach.StateMachine(outcomes=["exit"])
    SetPrint_sm.userdata.msg = "Message in user data"

    with SetPrint_sm:
        SetPrint_sm.add('Set', Set("Hello World"), transitions={"done": 'Print'},
                        remapping={"Set_msg_in": "msg", "Set_msg_out": "msg"})
        SetPrint_sm.add('Print', MessageReader(), transitions={"done": "exit"})

    return SetPrint_sm


class EmptyState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["continue"], input_keys=[], output_keys=[], io_keys=[])

    def execute(self, ud):
        rospy.sleep(2.0)
        return "continue"


# Fill in this function
def NestedStateMachine():
    main_sm = None

    return main_sm


def main():
    SimpleSM = NestedStateMachine()
    introspection_server = smach_ros.IntrospectionServer('SM', SimpleSM, '/SM_root')
    introspection_server.start()
    rospy.sleep(3.0)
    outcome = SimpleSM.execute()
    rospy.loginfo("Result : " + outcome)
    introspection_server.stop()

if __name__ == '__main__':
    rospy.init_node('tutorial_node')
    main()
    rospy.sleep(3.0)
