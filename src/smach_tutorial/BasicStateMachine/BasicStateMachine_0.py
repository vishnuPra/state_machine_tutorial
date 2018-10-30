#!/usr/bin/env python

import rospy
import smach
import smach_ros


class Set(smach.State):
    def __init__(self, msg=""):
        smach.State.__init__(self, outcomes=["done"], input_keys=["Set_msg_in"], output_keys=["Set_msg_out"])
        self.msg = msg

    def execute(self, ud):
        rospy.loginfo("Current userdata 'msg' : %s" % (ud.Set_msg_in))
        ud.Set_msg_out = self.msg
        rospy.sleep(2.0)
        return "done"


class MessageReader(smach.State):
    def __init__(self, msg=""):
        smach.State.__init__(self, outcomes=["done"], input_keys=["msg"])
        self.msg = msg

    def execute(self, ud):
        rospy.loginfo("Current userdata 'msg' : %s" % (ud.msg))
        rospy.sleep(2.0)
        return "done"


# Exercise 0: Function to create the state machine
def SetPrintStateMachine():
    # Initialize an empty State with a unique outcome: "exit"
    SetPrint_sm = smach.StateMachine(outcomes=["exit"])
    # Initialize a entry in the userdata "msg" with a initial value "Message in user data"
    SetPrint_sm.userdata.msg = "Message in user data"

    # Here we open the state machine, we can add states / or state machine inside.
    with SetPrint_sm:
        SetPrint_sm.add('Set', Set("Hello World"), transitions={"done": 'Print'},
                        remapping={"Set_msg_in": "msg", "Set_msg_out": "msg"})
        SetPrint_sm.add('Print', MessageReader(), transitions={"done": 'exit'})
    # Here we close the state machine, if there is something wrong in the transition,
    # then the check will tell us.

    # Then we return the created state machine as result of the function
    return SetPrint_sm


def main():
    SimpleSM = SetPrintStateMachine()
    introspection_server = smach_ros.IntrospectionServer('SM', SimpleSM, '/SM_root')
    introspection_server.start()
    rospy.sleep(3.0)
    outcome = SimpleSM.execute()
    rospy.loginfo("Result : " + outcome)
    introspection_server.stop()

if __name__ == '__main__':
    rospy.init_node('tutorial_node')
    main()
