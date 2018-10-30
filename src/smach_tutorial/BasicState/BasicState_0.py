#!/usr/bin/env python

import smach
import rospy
import smach_ros

# Exercise 0


class EmptyState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["continue"], input_keys=[], output_keys=[], io_keys=[])

    def execute(self, ud):
        rospy.sleep(2.0)
        return "continue"


# We define function 'WaitSM' to create a State Machine
def WaitSM():
    # Initialisation of an empty state machine with one outcome: 'exit'
    Wait_sm = smach.StateMachine(outcomes=["exit"])
    # 'with' means here: Open the state machine to let us add state and then close.
    with Wait_sm:
        # Here we add a EmptyState, with the name 'Wait1' and the transition
        # EmptyState.continue (outcome: 'continue') will go to the Wait2
        Wait_sm.add('Wait1', EmptyState(), transitions={"continue": 'Wait2'})
        # Here we add another state.
        Wait_sm.add('Wait2', EmptyState(), transitions={"continue": 'Wait3'})
        # The final state because his transitions is going to 'exit'
        Wait_sm.add('Wait3', EmptyState(), transitions={"continue": "exit"})

    return Wait_sm


# A function main to be run
def main():
    # Create a StateMachine with the previous function
    Wait_sm = WaitSM()
    # Setup  a IntrospectionServer to see in Smach Viewer
    introspection_server = smach_ros.IntrospectionServer('SM', Wait_sm, '/SM_root')
    # start the server
    introspection_server.start()
    # Wait  before the execution start
    rospy.sleep(3.0)
    # Start the state machine
    outcome = Wait_sm.execute()
    # Print a information about the result of the state machine
    rospy.loginfo("Result : " + outcome)
    # Stop the IntrospectionServer
    introspection_server.stop()


if __name__ == '__main__':
    # Initialialize a ROS Node
    rospy.init_node('tutorial_node')
    # Run the main function
    main()
