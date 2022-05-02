#! /usr/bin/python3 

import rospy
import actionlib
import spot_kinova_msgs.msg
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped
import numpy as np
import pinocchio as se3
import importlib, pkgutil
import threading
import cmd, sys, os
import copy

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

class ControlSuiteShell(cmd.Cmd):
    intro = bcolors.OKBLUE + "Welcome to the control suite shell.\nType help or ? to list commands.\n" + bcolors.ENDC
    prompt = "(csuite) "

    def __init__(self):
        cmd.Cmd.__init__(self)
        rospy.init_node('simulation_actions_client')
        self.joint_ctrl_client = actionlib.SimpleActionClient('/spot_kinova_action/joint_posture_control', spot_kinova_msgs.msg.JointPostureAction)
        self.joint_ctrl_client.wait_for_server()
        self.se3_ctrl_client = actionlib.SimpleActionClient('/spot_kinova_action/se3_control', spot_kinova_msgs.msg.SE3Action)
        self.se3_ctrl_client.wait_for_server()
        self.walk_client = actionlib.SimpleActionClient('/spot_kinova_action/move_base', spot_kinova_msgs.msg.WalkAction)
        self.walk_client.wait_for_server()
        self.body_posture_ctrl_client = actionlib.SimpleActionClient('/spot_kinova_action/body_posture_control', spot_kinova_msgs.msg.BodyPostureAction)
        self.body_posture_ctrl_client.wait_for_server()
        self.wholebody_ctrl_client = actionlib.SimpleActionClient('/spot_kinova_action/wholebody_control', spot_kinova_msgs.msg.WholebodyAction)
        self.wholebody_ctrl_client.wait_for_server()
        self.predefined_posture_ctrl_client = actionlib.SimpleActionClient('/spot_kinova_action/predefined_posture_control', spot_kinova_msgs.msg.PredefinedPostureAction)
        self.predefined_posture_ctrl_client.wait_for_server()

    def do_prehome(self, arg):
        'Go to the home position using predefined posture ctrl'
        goal = spot_kinova_msgs.msg.PredefinedPostureGoal
        goal.target_name = "NewHome"

        print ("anction sent")
        self.predefined_posture_ctrl_client.send_goal(goal)
        self.predefined_posture_ctrl_client.wait_for_result()
        if (self.predefined_posture_ctrl_client.get_result()):
            print ("action succeed")
        else:
            print ("action failed")

    def do_prefold(self, arg):
        'Go to the home position using predefined posture ctrl'
        goal = spot_kinova_msgs.msg.PredefinedPostureGoal
        goal.target_name = "NewTurnOff"

        print ("anction sent")
        self.predefined_posture_ctrl_client.send_goal(goal)
        self.predefined_posture_ctrl_client.wait_for_result()
        if (self.predefined_posture_ctrl_client.get_result()):
            print ("action succeed")
        else:
            print ("action failed")

    def do_home(self, arg):
        'Go to the home position using joint posture ctrl'
        goal = spot_kinova_msgs.msg.JointPostureGoal
        goal.duration = 2.0
        goal.target_joints = JointState()
        goal.target_joints.position = np.array([0.0, -40.0 / 180.0 * 3.14, 3.14, -100 / 180.0 * 3.14, 0, 60. / 180. * 3.14, 1.57])
        
        print ("anction sent")
        self.joint_ctrl_client.send_goal(goal)
        self.joint_ctrl_client.wait_for_result()
        if (self.joint_ctrl_client.get_result()):
            print ("action succeed")
        else:
            print ("action failed")
            
    def do_quit(self, arg):
        return True

if __name__ == '__main__':
    ControlSuiteShell().cmdloop()