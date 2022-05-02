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

    def do_fold(self, arg):
        'Go to the folding position using joint posture ctrl'
        goal = spot_kinova_msgs.msg.JointPostureGoal
        goal.duration = 2.0
        goal.target_joints = JointState()
        goal.target_joints.position = np.array([0.0, -2.0944, 3.14, -2.443, 0,0.3490, 1.57])

        print ("anction sent")
        self.joint_ctrl_client.send_goal(goal)
        self.joint_ctrl_client.wait_for_result()
        if (self.joint_ctrl_client.get_result()):
            print ("action succeed")
            self.do_body(True)
            self.do_walk(True)

        else:
            print ("action failed")

    def do_reach(self, arg):
        goal = spot_kinova_msgs.msg.SE3Goal
        goal.duration = 2.0
        goal.target_pose = Pose()
        goal.target_pose.position.x = -0.1
        goal.target_pose.position.y = -0.1
        goal.target_pose.position.z = 0.0

        goal.target_pose.orientation.x =  0.258819
        goal.target_pose.orientation.y = 0
        goal.target_pose.orientation.z = 0.
        goal.target_pose.orientation.w = 0.9659258

        goal.relative = True

        print ("anction sent")
        self.se3_ctrl_client.send_goal(goal)
        self.se3_ctrl_client.wait_for_result()
        if (self.se3_ctrl_client.get_result()):
            print ("action succeed")
        else:
            print ("action failed")

    def do_walk(self, arg):
       
        goal = spot_kinova_msgs.msg.WalkGoal

        goal.target_pose = PoseStamped()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = "odom"
        goal.target_pose.pose.position.x = 2.0
        goal.target_pose.pose.position.y = 0.0
        goal.target_pose.pose.position.z = 0.0

        goal.target_pose.pose.orientation.x = 0 
        goal.target_pose.pose.orientation.y = 0
        goal.target_pose.pose.orientation.z = 0.258819
        goal.target_pose.pose.orientation.w = 0.9659258
        goal.relative = True

        if (arg == True):
            goal.relative = False
            goal.target_pose.pose.position.x = 0.0
            goal.target_pose.pose.position.y = 0.0
            goal.target_pose.pose.position.z = 0.0

            goal.target_pose.pose.orientation.x = 0 
            goal.target_pose.pose.orientation.y = 0
            goal.target_pose.pose.orientation.z = 0
            goal.target_pose.pose.orientation.w = 1

        print ("anction sent")
        self.walk_client.send_goal(goal)
        self.walk_client.wait_for_result()

        if (self.walk_client.get_result()):
            print ("action succeed")
        else:
            print ("action failed")

    def do_body(self, arg):
        goal = spot_kinova_msgs.msg.BodyPostureGoal
        goal.duration = 2.0

        goal.target_pose = Pose()
        goal.target_pose.orientation.x =  0.258819
        goal.target_pose.orientation.y = 0
        goal.target_pose.orientation.z = 0.
        goal.target_pose.orientation.w = 0.9659258

        if (arg == True):
            goal.target_pose.orientation.x =  0.0
            goal.target_pose.orientation.y = 0
            goal.target_pose.orientation.z = 0.
            goal.target_pose.orientation.w = 1.0

        

        print ("anction sent")
        self.body_posture_ctrl_client.send_goal(goal)
        self.body_posture_ctrl_client.wait_for_result()
        if (self.body_posture_ctrl_client.get_result()):
            print ("action succeed")
        else:
            print ("action failed")

    def do_wholebody(self, arg):
        goal = spot_kinova_msgs.msg.WholebodyGoal
        goal.duration = 2.0

        goal.target_body_pose = Pose()
        goal.target_body_pose.orientation.x = 0.
        goal.target_body_pose.orientation.y = -0.258819
        goal.target_body_pose.orientation.z = 0.
        goal.target_body_pose.orientation.w = 0.9659258

        goal.target_ee_pose = Pose()
        goal.target_ee_pose.position.x = 0.2
        goal.target_ee_pose.position.y = -0.0
        goal.target_ee_pose.position.z = 0.1

        goal.target_ee_pose.orientation.x =  0.0
        goal.target_ee_pose.orientation.y = 0
        goal.target_ee_pose.orientation.z = 0.
        goal.target_ee_pose.orientation.w = 1.0

        goal.relative = True
        
        print ("anction sent")
        self.wholebody_ctrl_client.send_goal(goal)
        self.wholebody_ctrl_client.wait_for_result()
        if (self.wholebody_ctrl_client.get_result()):
            print ("action succeed")
        else:
            print ("action failed")


    def do_quit(self, arg):
        return True

if __name__ == '__main__':
    ControlSuiteShell().cmdloop()