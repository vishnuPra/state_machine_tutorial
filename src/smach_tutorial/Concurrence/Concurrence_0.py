#!/usr/bin/env python

import rospy
import smach
import random
import smach_ros

##Wait state for stating

class EmptyState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["continue"], input_keys=[], output_keys=[], io_keys=[])

    def execute(self, ud):
        rospy.sleep(2)
        return "continue"



##-----------------------------------------------------------------------------------
##Example 1
##-----------------------------------------------------------------------------------

class ConcurrenceState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["positiv", "negativ"])

    def execute(self, ud):
        rnumber = random.randint(-50,50)
        rospy.loginfo("Random : %d"%(rnumber))
        rospy.sleep(1.0)
        if(rnumber > 0):
           return "positiv"
        else:
           return "negativ"

def SimpleConcurrence():
    FooBar_cc = smach.Concurrence(outcomes = ["positiv","negativ"],
                                  default_outcome = "positiv",
                                  outcome_map = {"positiv" : {'Foo':"positiv",'Bar':"positiv"},
                                                 "negativ" : {'Foo':"negativ",'Bar':"negativ"}})
    with FooBar_cc:
        FooBar_cc.add('Foo',ConcurrenceState())
        FooBar_cc.add('Bar',ConcurrenceState())

    return FooBar_cc



def SimpleSM():
    Simple_sm = smach.StateMachine(outcomes=["exit"])

    with Simple_sm:
        Simple_sm.add('Wait', EmptyState(), transitions={"continue": 'Concurrence'})
        Simple_sm.add('Concurrence', SimpleConcurrence(), transitions={"positiv": 'exit',
                                                                     "negativ": 'exit'})

    return Simple_sm

def main():

    Simple_sm = SimpleSM()

    introspection_server = smach_ros.IntrospectionServer('SM', Simple_sm, '/SM_root')
    introspection_server.start()
    rospy.sleep(3.0)
    outcome = Simple_sm.execute()
    rospy.loginfo("Result : " + outcome)
    introspection_server.stop()


if __name__ == '__main__':
    rospy.init_node('tutorial_node')
    main()
