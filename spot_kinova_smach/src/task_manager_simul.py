#!/usr/bin/env python

from __future__ import print_function

import rospy
from smach import StateMachine
import smach_ros
import smach

from actionlib import *
from actionlib_msgs.msg import *
from smach_ros import SimpleActionState
from smach_ros import ServiceState
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import *
import numpy as np

from math import pi
from spot_kinova_msgs.msg import *

from states import StringTransitionState

def main():
    rospy.init_node('spot_kinova_smach')
    topic_name = '/spot_kinova/state_transition'
    issimulation = rospy.get_param("/issimulation")

    spot_kinova_sm = StateMachine(outcomes=['finished','aborted','preempted'])
    
    ## predefined goal (In simulation: fold - walking - ready - qr pick - pick up)
    spot_kinova_sm.userdata.fold_goal = JointPostureGoal(duration=2.0, target_joints=JointState(position=np.array([0.0, -2.0944, 3.14, -2.443, 0,0.3490, 1.57])))
    spot_kinova_sm.userdata.home_goal = JointPostureGoal(duration=2.0, target_joints=JointState(position=np.array([0.0, -40.0 / 180.0 * 3.14, 3.14, -100 / 180.0 * 3.14, 0, 60. / 180. * 3.14, 1.57])))
    spot_kinova_sm.userdata.ready_goal = JointPostureGoal(duration=2.0, target_joints=JointState(position=np.array([0.0, -90.0, 180.0, -90.0, 0, -90.0, 90.0]) * 3.14/180.0))
    walk_goal = spot_kinova_msgs.msg.WalkSimulationGoal()
    walk_goal.target_pose = PoseStamped()
    walk_goal.target_pose.header.stamp = rospy.Time.now()
    walk_goal.target_pose.header.frame_id = "odom"
    walk_goal.target_pose.pose.position.x = 2.0
    walk_goal.target_pose.pose.position.y = 0.0
    walk_goal.target_pose.pose.position.z = 0.0

    walk_goal.target_pose.pose.orientation.x = 0 
    walk_goal.target_pose.pose.orientation.y = 0
    walk_goal.target_pose.pose.orientation.z = 0.258819
    walk_goal.target_pose.pose.orientation.w = 0.9659258
    walk_goal.relative = True
    spot_kinova_sm.userdata.walk_goal = walk_goal
  
    ## defining state machine structure
    with spot_kinova_sm:
        StateMachine.add('START',
            StringTransitionState(topic_name, outcomes=['pick', 'ggds']), 
            transitions={'pick':'FOLD', 'ggds':'finished'})
        StateMachine.add('FOLD',
            SimpleActionState('/spot_kinova_action/joint_posture_control', JointPostureAction, goal=spot_kinova_sm.userdata.fold_goal),
            transitions={'succeeded':'WALK'})
        StateMachine.add('WALK',
            SimpleActionState('/spot_kinova_action/move_base', WalkSimulationAction, goal=spot_kinova_sm.userdata.walk_goal),
            transitions={'succeeded':'READY'})
        StateMachine.add('READY',
            SimpleActionState('/spot_kinova_action/joint_posture_control', JointPostureAction, goal=spot_kinova_sm.userdata.ready_goal),
            transitions={'succeeded':'finished'})

       
    # Run state machine introspection server
    intro_server = smach_ros.IntrospectionServer('spot_kinova', spot_kinova_sm,'/SPOT_KINOVA1')
    intro_server.start()
    spot_kinova_sm.execute()

    rospy.spin()

    rospy.signal_shutdown('All done.')

if __name__ == '__main__':
    main()
    
