#!/usr/bin/env python

import rospy
import smach
import smach_ros

# Exercice 1: define a state machine that go 3 time into the Foo state then leave

class Foo(smach.State):
    #each time that this state is executed it increased his inside counter by one
    def __init__(self):
        smach.State.__init__(self, outcomes=["continue", "out"])
        self.counter = 0

    def execute(self, ud):
        self.counter = self.counter + 1
        rospy.loginfo("Current Counter : %d" % (self.counter))
        rospy.sleep(2)
        if self.counter > 2:
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


# Fill in this function
def FooBarStateMachine():
    FooBar_sm = None #

    return FooBar_sm


def main():

    SimpleSM = FooBarStateMachine()
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
