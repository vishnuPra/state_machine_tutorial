#!/usr/bin/env python

import rospy
import smach

# Exercice 1

def FooBarStateMachine():
    FooBar_sm = smach.StateMachine(outcomes=["exit"])

    with FooBar_sm:
        FooBar_sm.add('Foo', Foo(), transitions={"continue": 'Bar',
                                                 "out": "exit"})
        FooBar_sm.add('Bar', Bar(), transitions={"continue" : 'Foo'})

    return FooBar_sm


# Exercice 2

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
