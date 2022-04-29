#! /usr/bin/python3 

import rospy
import actionlib
import spot_kinova_msgs.msg
from sensor_msgs.msg import JointState
import numpy as np

def joint_ctrl_client():
    client = actionlib.SimpleActionClient('/spot_kinova_action/joint_posture_control', spot_kinova_msgs.msg.JointPostureAction)
    client.wait_for_server()
    goal = spot_kinova_msgs.msg.JointPostureGoal
    goal.duration = 2.0
    goal.target_joints = JointState()
    goal.target_joints.position = np.array([0.0, -40.0 / 180.0 * 3.14, 3.14, -100 / 180.0 * 3.14, 0, 60. / 180. * 3.14, 1.57])
    
    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result()
    
if __name__ == '__main__':
    rospy.init_node('fibonacci_client_py')
    result = joint_ctrl_client()