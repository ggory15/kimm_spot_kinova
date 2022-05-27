#!/usr/bin/env python

import rospy
from smach import StateMachine
import smach_ros
import smach
from std_msgs.msg import String

class StringTransitionState(smach.State):
    def __init__(self, topic, outcomes=[], input_keys=[], output_keys=[]):
        self._topic = topic
        smach.State.__init__(self, outcomes, input_keys, output_keys)

    def execute(self, userdata):
        #print(self._topic)
        while rospy.is_shutdown() is False:
            #print('wait for message')
            trans_tag = rospy.wait_for_message(self._topic,String)
            #print(trans_tag.data)

            if trans_tag.data in self._outcomes:
                return trans_tag.data

class StringTransitionState(smach.State):
    def __init__(self, topic, outcomes=[], input_keys=[], output_keys=[]):
        self._topic = topic
        smach.State.__init__(self, outcomes, input_keys, output_keys)

    def execute(self, userdata):
        #print(self._topic)
        while rospy.is_shutdown() is False:
            #print('wait for message')
            trans_tag = rospy.wait_for_message(self._topic,String)
            #print(trans_tag.data)

            if trans_tag.data in self._outcomes:
                return trans_tag.data
