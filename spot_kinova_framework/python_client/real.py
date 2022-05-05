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
import time


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
        self.qr_walk_ctrl_client = actionlib.SimpleActionClient('/spot_kinova_action/qr_walk_control', spot_kinova_msgs.msg.QRWalkAction)
        self.qr_walk_ctrl_client.wait_for_server()
        self.gripper_ctrl_client = actionlib.SimpleActionClient('/spot_kinova_action/gripper_control', spot_kinova_msgs.msg.GripperAction)
        self.gripper_ctrl_client.wait_for_server()

    def do_prehome(self, arg):
        'Go to the home position using predefined posture ctrl'
        goal = spot_kinova_msgs.msg.PredefinedPostureGoal
        goal.posture_name = "NewHome"

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
        goal.posture_name = "NewTurnOff"

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
        goal.duration = 5.0
        goal.target_joints = JointState()
        goal.target_joints.position = np.array([0.0, -40.0 / 180.0 * 3.14, 3.14, -100 / 180.0 * 3.14, 0, 60. / 180. * 3.14, 1.57])
        
        print ("anction sent")
        self.joint_ctrl_client.send_goal(goal)
        self.joint_ctrl_client.wait_for_result()
        if (self.joint_ctrl_client.get_result()):
            print ("action succeed")
        else:
            print ("action failed")

    def do_reach(self, arg):
        goal = spot_kinova_msgs.msg.SE3Goal
        goal.duration = 3.0
        goal.target_pose = Pose()
        goal.target_pose.position.x = 0.1
        goal.target_pose.position.y = 0.1
        goal.target_pose.position.z = 0.0

        goal.target_pose.orientation.x =  0.0
        goal.target_pose.orientation.y = 0
        goal.target_pose.orientation.z = 0.
        goal.target_pose.orientation.w = 1

        goal.relative = True

        print ("anction sent")
        self.se3_ctrl_client.send_goal(goal)
        self.se3_ctrl_client.wait_for_result()
        if (self.se3_ctrl_client.get_result()):
            print ("action succeed")
        else:
            print ("action failed")

    def do_bodydown(self, arg):
        goal = spot_kinova_msgs.msg.BodyPostureGoal
        goal.duration =5.0

        goal.target_pose = Pose()
        goal.target_pose.orientation.x =  0.0
        goal.target_pose.orientation.y = 0.174
        goal.target_pose.orientation.z = 0.
        goal.target_pose.orientation.w = 0.985

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

    def do_bodyup(self, arg):
        goal = spot_kinova_msgs.msg.BodyPostureGoal
        goal.duration =5.0

        goal.target_pose = Pose()
        goal.target_pose.orientation.x =  0.0
        goal.target_pose.orientation.y = -0.174
        goal.target_pose.orientation.z = 0.
        goal.target_pose.orientation.w = 0.985

        if (arg == True):
            goal.target_pose.orientation.x =  0.0
            goal.target_pose.orientation.y = 0.
            goal.target_pose.orientation.z = 0.
            goal.target_pose.orientation.w = 1.0      

        print ("anction sent")
        self.body_posture_ctrl_client.send_goal(goal)
        self.body_posture_ctrl_client.wait_for_result()
        if (self.body_posture_ctrl_client.get_result()):
            print ("action succeed")
        else:
            print ("action failed")

    def do_bodyflat(self, arg):
        goal = spot_kinova_msgs.msg.BodyPostureGoal
        goal.duration =5.0

        goal.target_pose = Pose()
        goal.target_pose.orientation.x =  0.0
        goal.target_pose.orientation.y = 0.0
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
        goal.duration = 3.0

        goal.target_body_pose = Pose()
        goal.target_body_pose.orientation.x = 0.
        goal.target_body_pose.orientation.y = -0.174
        goal.target_body_pose.orientation.z = 0.
        goal.target_body_pose.orientation.w = 0.985

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

    def do_walk(self, arg):       
        goal = spot_kinova_msgs.msg.WalkGoal

        goal.target_pose = Pose()
        goal.target_pose.position.x = 1.16
        goal.target_pose.position.y = -0.8
        goal.target_pose.position.z = 0.0

        goal.target_pose.orientation.x = 0.0
        goal.target_pose.orientation.y = 0.0
        goal.target_pose.orientation.z = -0.3338069
        goal.target_pose.orientation.w = 0.9426415


        goal.relative = False

        print ("anction sent")
        self.walk_client.send_goal(goal)
        self.walk_client.wait_for_result()

        if (self.walk_client.get_result()):
            print ("action succeed")
        else:
            print ("action failed")
    
    def do_overall(self, arg):       
        'Go to the home position using predefined posture ctrl'
        goal = spot_kinova_msgs.msg.PredefinedPostureGoal
        goal.posture_name = "NewTurnOff"

        print ("anction sent")
        self.predefined_posture_ctrl_client.send_goal(goal)
        self.predefined_posture_ctrl_client.wait_for_result()
        if (self.predefined_posture_ctrl_client.get_result()):
            
            goal = spot_kinova_msgs.msg.WalkGoal

            goal.target_pose = Pose()
            goal.target_pose.position.x = 1.0
            goal.target_pose.position.y = -0.0
            goal.target_pose.position.z = 0.0

            goal.target_pose.orientation.x =  0.0
            goal.target_pose.orientation.y = 0
            goal.target_pose.orientation.z = 0.
            goal.target_pose.orientation.w = 1.0

            goal.relative = True

            print ("anction sent")
            self.walk_client.send_goal(goal)
            self.walk_client.wait_for_result()

            # if (self.walk_client.get_result()):
            #     time.sleep(1)
            #     goal = spot_kinova_msgs.msg.WholebodyGoal
            #     goal.duration = 3.0

            #     goal.target_body_pose = Pose()
            #     goal.target_body_pose.orientation.x = 0.
            #     goal.target_body_pose.orientation.y = -0.174
            #     goal.target_body_pose.orientation.z = 0.
            #     goal.target_body_pose.orientation.w = 0.985

            #     goal.target_ee_pose = Pose()
            #     goal.target_ee_pose.position.x = 0.2
            #     goal.target_ee_pose.position.y = -0.0
            #     goal.target_ee_pose.position.z = 0.1

            #     goal.target_ee_pose.orientation.x =  0.0
            #     goal.target_ee_pose.orientation.y = 0
            #     goal.target_ee_pose.orientation.z = 0.
            #     goal.target_ee_pose.orientation.w = 1.0

            #     goal.relative = True
                
            #     print ("anction sent")
            #     self.wholebody_ctrl_client.send_goal(goal)
            #     self.wholebody_ctrl_client.wait_for_result()
            #     if (self.wholebody_ctrl_client.get_result()):
            #         time.sleep(1)
            #         self.do_bodyflat(arg)

    def do_qrwalk(self, arg):
        'Walk toward the qr position'
        goal = spot_kinova_msgs.msg.QRGoal
        goal.topic_name = "aruco_markers_pose1"

        print ("anction sent")
        self.qr_walk_ctrl_client.send_goal(goal)
        self.qr_walk_ctrl_client.wait_for_result()
    
    def do_open(self, arg):
        'Open Gripper'
        goal = spot_kinova_msgs.msg.GripperGoal
        goal.open = True

        print ("anction sent")
        self.gripper_ctrl_client.send_goal(goal)
        self.gripper_ctrl_client.wait_for_result()

        if (self.gripper_ctrl_client.get_result()):
            print ("action succeed")
        else:
            print ("action failed")
    
    
    def do_close(self, arg):
        'Open Gripper'
        goal = spot_kinova_msgs.msg.GripperGoal
        goal.open = False

        print ("anction sent")
        self.gripper_ctrl_client.send_goal(goal)
        self.gripper_ctrl_client.wait_for_result()

        if (self.gripper_ctrl_client.get_result()):
            print ("action succeed")
        else:
            print ("action failed")
    

    def do_quit(self, arg):
        return True

if __name__ == '__main__':
    ControlSuiteShell().cmdloop()
