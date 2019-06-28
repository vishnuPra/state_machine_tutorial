#!/usr/bin/env python

import smach
import rospy
import smach_ros

# Exercise 1: Create a Wait State with userdata

class WaitState(smach.State):
    def __init__(self):
        # Setup the smach.State
        smach.State.__init__(self, outcomes=["continue"], input_keys=["sleep_time"], output_keys=[], io_keys=[])
        pass

    def execute(self, ud):
        rospy.sleep(ud.sleep_time)
        return "continue"

# Create the State Machine
def WaitSM():
    Wait_sm = smach.StateMachine(outcomes=["exit"])
    # Here we create and initialialize the userdata to be use in the state machine
    Wait_sm.userdata.sleep_time = 2.0

    rospy.logwarn("Sleep time set to %s" % str(Wait_sm.userdata.sleep_time))
    with Wait_sm:
        Wait_sm.add('Wait1', WaitState(), transitions={"continue": 'Wait2'})
        Wait_sm.add('Wait2', WaitState(), transitions={"continue": 'Wait3'})
        Wait_sm.add('Wait3', WaitState(), transitions={"continue": "exit"})

    return Wait_sm


def main():
    Wait_sm = WaitSM()
    introspection_server = smach_ros.IntrospectionServer('SM', Wait_sm, '/SM_root')
    introspection_server.start()
    rospy.sleep(3.0)
    outcome = Wait_sm.execute()
    rospy.loginfo("Result : " + outcome)
    introspection_server.stop()

if __name__ == '__main__':
    rospy.init_node('tutorial_node')
    main()
