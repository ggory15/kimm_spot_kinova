#! /usr/bin/python3

import rospy
import actionlib
import spot_kinova_msgs.msg
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from std_msgs.msg import Float32MultiArray, String
from std_srvs.srv import Trigger
from spot_msgs.srv import ListGraph, DownloadGraph, UploadGraph, SetLocalizationFiducial
import spot_msgs.msg
import numpy as np
import importlib, pkgutil
import threading
import cmd, sys, os
import copy
import time
import math

from nav_msgs.msg import Odometry
import json

def make_dir(path):
    if not os.path.exists(path):
        os.makedirs(path)
        print(path, 'directory is created.')

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
        self.se3_array_ctrl_client = actionlib.SimpleActionClient('/spot_kinova_action/se3_array_control', spot_kinova_msgs.msg.SE3ArrayAction)
        self.se3_array_ctrl_client.wait_for_server()
        self.qr_pick_ctrl_client = actionlib.SimpleActionClient('/spot_kinova_action/qr_pick_control', spot_kinova_msgs.msg.QRPickAction)
        self.qr_pick_ctrl_client.wait_for_server()

        self.navigate_to_client = actionlib.SimpleActionClient('/spot/navigate_to', spot_msgs.msg.NavigateToAction)
        self.navigate_to_client.wait_for_server()

        self.waypoint_ids = []
        self.map_root = os.path.join(os.path.expanduser('~'),'.kimm_map')

        self.issucceed = True

        self.mech_pose = Pose()
        self.iscallback_mech = False 
        self.isgoodpose = False                   
        self.mech_call_pub = rospy.Publisher('/mech_call', String, queue_size=10)
        rospy.Subscriber('mechmind_publisher/pose', Pose, self.mechmind_pose_callback)
        
        self.qr1_flag = True
        self.qr2_flag = True
        self.qr3_flag = True
        self.qr4_flag = True
        self.qr5_flag = True
        self.qr4_spot_flag = True
        self.qr6_spot_flag = True
        rospy.Subscriber('aruco_markers_pose1', Pose, self.qr1_callback)
        rospy.Subscriber('aruco_markers_pose2', Pose, self.qr2_callback)
        rospy.Subscriber('aruco_markers_pose3', Pose, self.qr3_callback)
        rospy.Subscriber('aruco_markers_pose4', Pose, self.qr4_callback)
        rospy.Subscriber('aruco_markers_pose5', Pose, self.qr5_callback)                            
        rospy.Subscriber('aruco_markers_pose4_spot', Pose, self.qr4_spot_callback)                    
        rospy.Subscriber('aruco_markers_pose6_spot', Pose, self.qr6_spot_callback)                    

        self.gripper_change = False

        if (self.gripper_change):
            self.changed_gripper_offset = -0.05
        else:
            self.changed_gripper_offset = 0.0
        

        self.gripper_length = 0.2        

        self.box_qr2_to_grasp_x = 0.0   #0.01
        self.box_qr2_to_grasp_y = -0.09  #-0.068                                                  

        self.tray_qr5_to_grasp_x =   0.0#-0.005
        self.tray_qr5_to_grasp_y =  -0.065#-0.052
        self.tray_qr5_to_plant1_x = self.changed_gripper_offset - 0.077 #-0.075
        self.tray_qr5_to_plant1_y =  0.075
        self.tray_qr5_to_plant2_x = self.changed_gripper_offset - 0.077   
        self.tray_qr5_to_plant2_y = -0.065#-0.065

        self.table1_qr3_to_plant1_x = self.changed_gripper_offset - 0.075 #-0.08
        self.table1_qr3_to_plant1_y =  0.075 #0.065
        self.table1_qr3_to_plant2_x = self.changed_gripper_offset - 0.075#-0.08
        self.table1_qr3_to_plant2_y = -0.06#-0.07

        self.table2_qr4_to_plant1_x = self.changed_gripper_offset - 0.075
        self.table2_qr4_to_plant1_y =  0.07
        self.table2_qr4_to_plant2_x = self.changed_gripper_offset - 0.075
        self.table2_qr4_to_plant2_y = -0.065        

        self.pick_x_offset = -0.05

        self.map_root = os.path.join(os.path.expanduser('~'),'.kimm_map')
        rospy.Subscriber('/spot/odometry', Odometry, self.odom_callback)
        self.odometry = Odometry()
        self.pose_goals = {}   

    #########################################################################################################################
    ############################################## spot navigation ##########################################################
    #########################################################################################################################
    def odom_callback(self, msg):
        self.odometry = msg

    def do_add_goals_from_odom(self, arg):
        self.pose_goals[arg] = [self.odometry.pose.pose.position.x,self.odometry.pose.pose.position.y,self.odometry.pose.pose.position.z,self.odometry.pose.pose.orientation.x, self.odometry.pose.pose.orientation.y, self.odometry.pose.pose.orientation.z, self.odometry.pose.pose.orientation.w]
        print(self.pose_goals)

    def do_walk_to_goal(self, arg):
        try:
            pose_goal = self.pose_goals[arg]
            print(pose_goal)

            goal = spot_kinova_msgs.msg.WalkGoal

            goal.target_pose = Pose()
            goal.target_pose.position.x = pose_goal[0]#pose_goal.position.x
            goal.target_pose.position.y = pose_goal[1]#pose_goal.position.y
            goal.target_pose.position.z = pose_goal[2]#pose_goal.position.z

            goal.target_pose.orientation.x = pose_goal[3]#pose_goal.orientation.x
            goal.target_pose.orientation.y = pose_goal[4]#pose_goal.orientation.y
            goal.target_pose.orientation.z = pose_goal[5]#pose_goal.orientation.z
            goal.target_pose.orientation.w = pose_goal[6]#pose_goal.orientation.w

            goal.relative = False

            print ("action sent")
            self.walk_client.send_goal(goal)
            self.walk_client.wait_for_result()

            if (self.walk_client.get_result()):
                print ("action succeed")
            else:
                print ("action failed")
        except:
            print('except')


    def do_save_goals(self, arg):
        'Save goals'

        make_dir(self.map_root)
        json_path = os.path.join(self.map_root,'goals.json')

        with open(json_path,'w') as f:
            json_val = json.dump(self.pose_goals, f, indent=4)

    def do_load_goals(self, arg):
        'Load goals'

        make_dir(self.map_root)
        json_path = os.path.join(self.map_root,'goals.json')

        with open(json_path, 'r') as f:
            self.pose_goals = json.load(f)

            print(self.pose_goals)

    def do_clear_goals(self, arg):
        'Clear golas'

        self.pose_goals = dict()
        print(self.pose_goals)

    def do_remove_goal(self, arg):
        'Remove goal from keyvalue'

        try:
            del self.pose_goals[arg]
            print(self.pose_goals)
        except:
            print('raise exception')

    #########################################################################################################################
    ################################################   mechmind   ###########################################################
    #########################################################################################################################    
    def do_mechcall(self, arg):
        'call the pose from the mechmind with designated object label'
        
        self.isgoodpose = False
        
        object_number = int(arg)
        
        if(object_number >= 0 and object_number < 11):
            label = str(arg)
            mech_msg = String()
            mech_msg.data = "mech_call_" + label #request designated labeled object pose
            print ("object_number is", object_number)
            self.mech_call_pub.publish(mech_msg)
        else:
            print("object number range is 0 ~ 10, try again")

    def mechmind_pose_callback(self, msg):
        self.mech_pose = msg
        self.iscallback_mech = True
        
        print(self.mech_pose)      

        if (self.mech_pose.position.x > 0.18 or self.mech_pose.position.x < 0.0):
            print ("target object's x position is not suitable to grasp")                
            self.isgoodpose = False            

        elif (math.fabs(self.mech_pose.position.y) < 0.1 or math.fabs(self.mech_pose.position.y) > 0.22):
            print ("target object's y position is not suitable to grasp")
            self.isgoodpose = False            

        else:
            print ("pose received, do mechqrpick!")
            self.isgoodpose = True

    #########################################################################################################################
    ################################################      qr     ############################################################
    #########################################################################################################################        
    def qr1_callback(self, msg):
        if (not self.qr1_flag):
            self.qr1_pose = msg
            self.qr1_flag = True
            print (self.qr1_pose)

    def qr2_callback(self, msg):
        if (not self.qr2_flag):
            self.qr2_pose = msg
            self.qr2_flag = True
            print (self.qr2_pose)

    def qr3_callback(self, msg):
        if (not self.qr3_flag):
            self.qr3_pose = msg
            self.qr3_flag = True
            print (self.qr3_pose)

    def qr4_callback(self, msg):
        if (not self.qr4_flag):
            self.qr4_pose = msg
            self.qr4_flag = True
            print (self.qr4_pose)

    def qr5_callback(self, msg):
        if (not self.qr5_flag):
            self.qr5_pose = msg
            self.qr5_flag = True
            print (self.qr5_pose)

    def qr4_spot_callback(self, msg):
        if (not self.qr4_spot_flag):
            self.qr4_spot_pose = msg
            self.qr4_spot_flag = True
            print (self.qr4_spot_pose)

    def qr6_spot_callback(self, msg):
        if (not self.qr6_spot_flag):
            self.qr6_spot_pose = msg
            self.qr6_spot_flag = True
            print (self.qr6_spot_pose)

    def do_qr1save(self, arg):
        self.qr1_flag = False

    def do_qr2save(self, arg):
        self.qr2_flag = False

    def do_qr3save(self, arg):
        self.qr3_flag = False

    def do_qr4save(self, arg):
        self.qr4_flag = False

    def do_qr5save(self, arg):
        self.qr5_flag = False        

    def do_qr4_spot_save(self, arg):
        self.qr4_spot_flag = False        

    def do_qr6_spot_save(self, arg):
        self.qr6_spot_flag = False        
    
    #########################################################################################################################
    ############################################## kinova primitives ########################################################
    #########################################################################################################################                    
    
    ################################################## object pnp ###########################################################                    
    def do_arm_mechpose(self, arg): #not used for now
        'Go to the up position for mechcall'

        self.issucceed = False

        goal = spot_kinova_msgs.msg.JointPostureGoal
        goal.duration = 5.0
        goal.target_joints = JointState()        
        goal.target_joints.position = np.array([0.0, -2.18, 3.14, -2.443, 0, 1.83, 1.57])        

        print ("action sent")
        self.joint_ctrl_client.send_goal(goal)

        self.joint_ctrl_client.wait_for_result()
        if (self.joint_ctrl_client.get_result()):
            print ("action succeed")
            self.issucceed = True            
        else:
            print ("action failed")
            self.issucceed = False

    def do_mechqrpick(self, arg):
        'pick the designated labeled object by aruco tag recognition'

        self.issucceed = False

        goal = spot_kinova_msgs.msg.QRPickGoal

        # goal.topic_name = "aruco_markers_pose1"
        goal.qr_pose = self.qr1_pose
        goal.approach_offset = 0.2 
        
        goal.target_pose = Pose()
        if (self.iscallback_mech):
            print ("go to mech pose")            

            goal.target_pose.position.x    =  self.mech_pose.position.x
            goal.target_pose.position.y    =  self.mech_pose.position.y
            # goal.target_pose.position.z    =  self.mech_pose.position.z + self.gripper_length + 0.05
            goal.target_pose.position.z    =  self.gripper_length + 0.01
            goal.target_pose.orientation.x =  0.0
            goal.target_pose.orientation.y =  0.0
            goal.target_pose.orientation.z =  0.7071
            goal.target_pose.orientation.w =  0.7071     

            self.iscallback_mech = False

            goal.duration = 3.0

            print ("action sent")
            self.qr_pick_ctrl_client.send_goal(goal) 

            self.qr_pick_ctrl_client.wait_for_result()
            if (self.qr_pick_ctrl_client.get_result()):
                print ("action succeed")                
                self.issucceed = True
            else:
                print ("action failed")
                self.issucceed = False

        else:
            print ("mech pose is not received, try mechcall again")

    def do_qrpick(self, arg): #test function
        'pick example without mechmind'

        self.issucceed = False

        goal = spot_kinova_msgs.msg.QRPickGoal

        # goal.topic_name = "aruco_markers_pose1"        
        goal.qr_pose = self.qr1_pose
        goal.approach_offset = 0.2 
        
        goal.target_pose = Pose()                                
        goal.target_pose.position.x    =  0.0733101729577463
        goal.target_pose.position.y    =  0.15716984563429737        
        goal.target_pose.position.z    =  self.gripper_length + 0.130
        goal.target_pose.orientation.x =  0.0
        goal.target_pose.orientation.y =  0.0
        goal.target_pose.orientation.z =  0.7071
        goal.target_pose.orientation.w =  0.7071

        goal.duration = 5.0

        print ("action sent")
        self.qr_pick_ctrl_client.send_goal(goal) 

        self.qr_pick_ctrl_client.wait_for_result()
        if (self.qr_pick_ctrl_client.get_result()):
            print ("action succeed")                
            self.issucceed = True
        else:
            print ("action failed")
            self.issucceed = False

    def do_pickup(self, arg):
        goal = spot_kinova_msgs.msg.SE3ArrayGoal

        self.issucceed = False

        goal.relative = True
        goal.target_poses = PoseArray()
        pose_se = Pose()
        pose_se.position.x = 0.0
        pose_se.position.y = 0.0
        pose_se.position.z = 0.20
        pose_se.orientation.x =  0.0
        pose_se.orientation.y = 0.0
        pose_se.orientation.z = 0.0
        pose_se.orientation.w = 1.0
        goal.target_poses.poses.append(pose_se)
        

        goal.durations = Float32MultiArray()
        goal.durations.data.append(2.0)        
        
        print ("action sent")        
        self.se3_array_ctrl_client.send_goal(goal)

        self.se3_array_ctrl_client.wait_for_result()
        if (self.se3_array_ctrl_client.get_result()):
            print ("action succeed")
            self.issucceed = True
        else:
            print ("action failed")
            self.issucceed = False

    def do_gotobox(self, arg):
        'go to the box w.r.t qr1_pose to place the object'

        self.issucceed = False

        goal = spot_kinova_msgs.msg.QRPickGoal

        # goal.topic_name = "aruco_markers_pose1"        
        goal.qr_pose = self.qr1_pose
        goal.approach_offset = 0.05
        
        goal.target_pose = Pose()                                
        goal.target_pose.position.x    =  0.05
        goal.target_pose.position.y    =  -0.35
        goal.target_pose.position.z    =  self.gripper_length + 0.25
        goal.target_pose.orientation.x =  0.0
        goal.target_pose.orientation.y =  0.0
        goal.target_pose.orientation.z =  0.7071
        goal.target_pose.orientation.w =  0.7071

        goal.duration = 6.0

        print ("action sent")
        self.qr_pick_ctrl_client.send_goal(goal) 

        self.qr_pick_ctrl_client.wait_for_result()
        if (self.qr_pick_ctrl_client.get_result()):
            print ("action succeed")                
            self.issucceed = True
        else:
            print ("action failed")
            self.issucceed = False

    ################################################## box pnp ###########################################################        
    def do_qrboxpick(self, arg):
        'pick the box w.r.t. qr marker'

        self.issucceed = False

        goal = spot_kinova_msgs.msg.QRPickGoal

        # goal.topic_name = "aruco_markers_pose2"
        goal.qr_pose = self.qr2_pose
        
        goal.target_pose = Pose()                
        goal.approach_offset = 0.2

        goal.target_pose.position.x    =                        self.box_qr2_to_grasp_x
        goal.target_pose.position.y    =                        self.box_qr2_to_grasp_y
        goal.target_pose.position.z    =  self.gripper_length - 0.05
        goal.target_pose.orientation.x =  0.0
        goal.target_pose.orientation.y =  0.0
        goal.target_pose.orientation.z =  0.0
        goal.target_pose.orientation.w =  1.0        

        goal.duration = 3.0        

        print ("action sent")
        self.qr_pick_ctrl_client.send_goal(goal) 

        self.qr_pick_ctrl_client.wait_for_result()
        if (self.qr_pick_ctrl_client.get_result()):
            print ("action succeed")                
            self.issucceed = True
        else:
            print ("action failed")
            self.issucceed = False  

    def do_qrboxplace(self, arg):
        'place the box w.r.t. qr marker'

        self.issucceed = False

        goal = spot_kinova_msgs.msg.QRPickGoal

        # goal.topic_name = "aruco_markers_pose6_spot"
        goal.qr_pose = self.qr6_spot_pose
        
        goal.target_pose = Pose()                
        goal.approach_offset = 0.1

        goal.target_pose.position.x    =                        0.2
        goal.target_pose.position.y    =                        0.2
        goal.target_pose.position.z    =  self.gripper_length - 0.05
        goal.target_pose.orientation.x =  0.0
        goal.target_pose.orientation.y =  0.0
        goal.target_pose.orientation.z =  0.0
        goal.target_pose.orientation.w =  1.0        

        goal.duration = 3.0        

        print ("action sent")
        self.qr_pick_ctrl_client.send_goal(goal) 

        self.qr_pick_ctrl_client.wait_for_result()
        if (self.qr_pick_ctrl_client.get_result()):
            print ("action succeed")                
            self.issucceed = True
        else:
            print ("action failed")
            self.issucceed = False  

    def do_goto_boxplace(self, arg):
        goal = spot_kinova_msgs.msg.SE3ArrayGoal

        self.issucceed = False

        goal.relative = True
        goal.target_poses = PoseArray()
        pose_se = Pose()
        pose_se.position.x =  0.25
        pose_se.position.y =  0.05
        pose_se.position.z =  0.0
        pose_se.orientation.x = 0.0
        pose_se.orientation.y = 0.0
        pose_se.orientation.z = 0.0
        pose_se.orientation.w = 1.0
        goal.target_poses.poses.append(pose_se)
        

        goal.durations = Float32MultiArray()
        goal.durations.data.append(3.0)        
        
        print ("action sent")        
        self.se3_array_ctrl_client.send_goal(goal)

        self.se3_array_ctrl_client.wait_for_result()
        if (self.se3_array_ctrl_client.get_result()):
            print ("action succeed")
            self.issucceed = True
        else:
            print ("action failed")
            self.issucceed = False

    ################################################## tray pnp ###########################################################                        
    def do_qrtraypick(self, arg):
        'pick the tray w.r.t. qr marker'

        self.issucceed = False

        goal = spot_kinova_msgs.msg.QRPickGoal

        # goal.topic_name = "aruco_markers_pose5"
        goal.qr_pose = self.qr5_pose
        
        goal.target_pose = Pose()                
        goal.approach_offset = 0.1

        goal.target_pose.position.x    =                        self.tray_qr5_to_grasp_x                         
        goal.target_pose.position.y    =                        self.tray_qr5_to_grasp_y                        
        goal.target_pose.position.z    =  self.gripper_length - 0.05
        goal.target_pose.orientation.x =  0.0
        goal.target_pose.orientation.y =  0.0
        goal.target_pose.orientation.z =  0.0
        goal.target_pose.orientation.w =  1.0        

        goal.duration = 3.0        

        print ("action sent")
        self.qr_pick_ctrl_client.send_goal(goal) 

        self.qr_pick_ctrl_client.wait_for_result()
        if (self.qr_pick_ctrl_client.get_result()):
            print ("action succeed")                
            self.issucceed = True
        else:
            print ("action failed")
            self.issucceed = False    

    def do_qrtraypick_changed_gripper(self, arg):
        'pick the tray w.r.t. qr marker'

        self.issucceed = False

        goal = spot_kinova_msgs.msg.QRPickGoal

        # goal.topic_name = "aruco_markers_pose5"
        goal.qr_pose = self.qr5_pose
        
        goal.target_pose = Pose()                
        goal.approach_offset = 0.1

        goal.target_pose.position.x    =                      - 0.20
        goal.target_pose.position.y    =                        0.0
        goal.target_pose.position.z    =  self.gripper_length - 0.045
        goal.target_pose.orientation.x =  0.0
        goal.target_pose.orientation.y =  0.0
        goal.target_pose.orientation.z =  1.0
        goal.target_pose.orientation.w =  0.0        

        goal.duration = 3.0        

        print ("action sent")
        self.qr_pick_ctrl_client.send_goal(goal) 

        self.qr_pick_ctrl_client.wait_for_result()
        if (self.qr_pick_ctrl_client.get_result()):
            print ("action succeed")                
            self.issucceed = True
        else:
            print ("action failed")
            self.issucceed = False    

    def do_goto_trayplace(self, arg):
        goal = spot_kinova_msgs.msg.SE3ArrayGoal

        self.issucceed = False

        goal.relative = True
        goal.target_poses = PoseArray()
        pose_se = Pose()
        pose_se.position.x =  0.25
        pose_se.position.y =  0.05
        pose_se.position.z =  0.0
        pose_se.orientation.x = 0.0
        pose_se.orientation.y = 0.0
        pose_se.orientation.z = 0.0
        pose_se.orientation.w = 1.0
        goal.target_poses.poses.append(pose_se)
        

        goal.durations = Float32MultiArray()
        goal.durations.data.append(3.0)        
        
        print ("action sent")        
        self.se3_array_ctrl_client.send_goal(goal)

        self.se3_array_ctrl_client.wait_for_result()
        if (self.se3_array_ctrl_client.get_result()):
            print ("action succeed")
            self.issucceed = True
        else:
            print ("action failed")
            self.issucceed = False

    def do_qrtrayplace(self, arg):
        'place the box w.r.t. qr marker'

        self.issucceed = False

        goal = spot_kinova_msgs.msg.QRPickGoal

        # goal.topic_name = "aruco_markers_pose4_spot"
        goal.qr_pose = self.qr4_spot_pose
        
        goal.target_pose = Pose()                
        goal.approach_offset = 0.1

        goal.target_pose.position.x    =                        0.2
        goal.target_pose.position.y    =                        0.2
        goal.target_pose.position.z    =  self.gripper_length - 0.05
        goal.target_pose.orientation.x =  0.0
        goal.target_pose.orientation.y =  0.0
        goal.target_pose.orientation.z =  0.0
        goal.target_pose.orientation.w =  1.0        

        goal.duration = 3.0        

        print ("action sent")
        self.qr_pick_ctrl_client.send_goal(goal) 

        self.qr_pick_ctrl_client.wait_for_result()
        if (self.qr_pick_ctrl_client.get_result()):
            print ("action succeed")                
            self.issucceed = True
        else:
            print ("action failed")
            self.issucceed = False  
        
    ################################################## plant table #1 to tray ###########################################################                                
    def do_qrplant1pick_fromtable1(self, arg):
        'pick the plant1 w.r.t. qr marker'

        self.issucceed = False

        goal = spot_kinova_msgs.msg.QRPickGoal

        # goal.topic_name = "aruco_markers_pose3"
        goal.qr_pose = self.qr3_pose
        
        goal.target_pose = Pose()                
        goal.approach_offset = 0.1 

        goal.target_pose.position.x    =                        self.table1_qr3_to_plant1_x + self.pick_x_offset
        goal.target_pose.position.y    =                        self.table1_qr3_to_plant1_y
        goal.target_pose.position.z    =  self.gripper_length + 0.01#0.035 
        
        if (self.gripper_change):
            goal.target_pose.orientation.x =  0.0
            goal.target_pose.orientation.y =  0.0
            goal.target_pose.orientation.z =  1.0
            goal.target_pose.orientation.w =  0.0        
        else:
            goal.target_pose.orientation.x =  0.0
            goal.target_pose.orientation.y =  0.0
            goal.target_pose.orientation.z =  0.0
            goal.target_pose.orientation.w =  1.0

        goal.duration = 3.0        

        print ("action sent")
        self.qr_pick_ctrl_client.send_goal(goal) 

        self.qr_pick_ctrl_client.wait_for_result()
        if (self.qr_pick_ctrl_client.get_result()):
            print ("action succeed")                
            self.issucceed = True
        else:
            print ("action failed")
            self.issucceed = False 

    def do_qrtray1plantapproach(self, arg):
        'place the plant to the tray1 position w.r.t. qr marker'

        self.issucceed = False

        goal = spot_kinova_msgs.msg.QRPickGoal

        # goal.topic_name = "aruco_markers_pose5"
        goal.qr_pose = self.qr5_pose
        
        goal.target_pose = Pose()                
        goal.approach_offset = 0.1

        goal.target_pose.position.x    =                        self.tray_qr5_to_plant1_x
        goal.target_pose.position.y    =                        self.tray_qr5_to_plant1_y
        goal.target_pose.position.z    =  self.gripper_length + 0.025#0.04
        
        if (self.gripper_change):
            goal.target_pose.orientation.x =  0.0
            goal.target_pose.orientation.y =  0.0
            goal.target_pose.orientation.z =  1.0
            goal.target_pose.orientation.w =  0.0        
        else:
            goal.target_pose.orientation.x =  0.0
            goal.target_pose.orientation.y =  0.0
            goal.target_pose.orientation.z =  0.0
            goal.target_pose.orientation.w =  1.0

        goal.duration = 3.0        

        print ("action sent")
        self.qr_pick_ctrl_client.send_goal(goal) 

        self.qr_pick_ctrl_client.wait_for_result()
        if (self.qr_pick_ctrl_client.get_result()):
            print ("action succeed")                
            self.issucceed = True
        else:
            print ("action failed")
            self.issucceed = False    

    def do_qrplant2pick_fromtable1(self, arg):
        'pick the plant2 w.r.t. qr marker'

        self.issucceed = False

        goal = spot_kinova_msgs.msg.QRPickGoal

        # goal.topic_name = "aruco_markers_pose3"
        goal.qr_pose = self.qr3_pose
        
        goal.target_pose = Pose()                
        goal.approach_offset = 0.1

        goal.target_pose.position.x    =                        self.table1_qr3_to_plant2_x + self.pick_x_offset
        goal.target_pose.position.y    =                        self.table1_qr3_to_plant2_y
        goal.target_pose.position.z    =  self.gripper_length + 0.010#0.035
        
        if (self.gripper_change):
            goal.target_pose.orientation.x =  0.0
            goal.target_pose.orientation.y =  0.0
            goal.target_pose.orientation.z =  1.0
            goal.target_pose.orientation.w =  0.0        
        else:
            goal.target_pose.orientation.x =  0.0
            goal.target_pose.orientation.y =  0.0
            goal.target_pose.orientation.z =  0.0
            goal.target_pose.orientation.w =  1.0        

        goal.duration = 3.0        

        print ("action sent")
        self.qr_pick_ctrl_client.send_goal(goal) 

        self.qr_pick_ctrl_client.wait_for_result()
        if (self.qr_pick_ctrl_client.get_result()):
            print ("action succeed")                
            self.issucceed = True
        else:
            print ("action failed")
            self.issucceed = False  
 
    def do_qrtray2plantapproach(self, arg):
        'place the plant to the tray2 position w.r.t. qr marker'

        self.issucceed = False

        goal = spot_kinova_msgs.msg.QRPickGoal

        # goal.topic_name = "aruco_markers_pose5"
        goal.qr_pose = self.qr5_pose
        
        goal.target_pose = Pose()                
        goal.approach_offset = 0.1 

        goal.target_pose.position.x    =                        self.tray_qr5_to_plant2_x
        goal.target_pose.position.y    =                        self.tray_qr5_to_plant2_y
        goal.target_pose.position.z    =  self.gripper_length + 0.025#0.04
        
        if (self.gripper_change):
            goal.target_pose.orientation.x =  0.0
            goal.target_pose.orientation.y =  0.0
            goal.target_pose.orientation.z =  1.0
            goal.target_pose.orientation.w =  0.0        
        else:
            goal.target_pose.orientation.x =  0.0
            goal.target_pose.orientation.y =  0.0
            goal.target_pose.orientation.z =  0.0
            goal.target_pose.orientation.w =  1.0        

        goal.duration = 3.0        

        print ("action sent")
        self.qr_pick_ctrl_client.send_goal(goal) 

        self.qr_pick_ctrl_client.wait_for_result()
        if (self.qr_pick_ctrl_client.get_result()):
            print ("action succeed")                
            self.issucceed = True
        else:
            print ("action failed")
            self.issucceed = False 
            
    def do_goto_tableqr(self, arg):
        goal = spot_kinova_msgs.msg.SE3ArrayGoal

        self.issucceed = False

        goal.relative = True
        goal.target_poses = PoseArray()
        pose_se = Pose()
        pose_se.position.x =  0.0
        pose_se.position.y =  0.15
        pose_se.position.z = -0.09
        pose_se.orientation.x = 0.0
        pose_se.orientation.y = 0.0
        pose_se.orientation.z = 0.0
        pose_se.orientation.w = 1.0
        goal.target_poses.poses.append(pose_se)
        

        goal.durations = Float32MultiArray()
        goal.durations.data.append(3.0)        
        
        print ("action sent")        
        self.se3_array_ctrl_client.send_goal(goal)

        self.se3_array_ctrl_client.wait_for_result()
        if (self.se3_array_ctrl_client.get_result()):
            print ("action succeed")
            self.issucceed = True
        else:
            print ("action failed")
            self.issucceed = False 

    ################################################## plant tray to table #2 #############################################                            
    def do_qrplant1pick_fromtray(self, arg):
        'pick the plant1 w.r.t. qr marker'

        self.issucceed = False

        goal = spot_kinova_msgs.msg.QRPickGoal

        # goal.topic_name = "aruco_markers_pose5"
        goal.qr_pose = self.qr5_pose
        
        goal.target_pose = Pose()                
        goal.approach_offset = 0.1
        
        goal.target_pose.position.x    =                        self.tray_qr5_to_plant1_x + self.pick_x_offset
        goal.target_pose.position.y    =                        self.tray_qr5_to_plant1_y
        goal.target_pose.position.z    =  self.gripper_length + 0.015
        
        if (self.gripper_change):
            goal.target_pose.orientation.x =  0.0
            goal.target_pose.orientation.y =  0.0
            goal.target_pose.orientation.z =  1.0
            goal.target_pose.orientation.w =  0.0        
        else:
            goal.target_pose.orientation.x =  0.0
            goal.target_pose.orientation.y =  0.0
            goal.target_pose.orientation.z =  0.0
            goal.target_pose.orientation.w =  1.0        

        goal.duration = 3.0        

        print ("action sent")
        self.qr_pick_ctrl_client.send_goal(goal) 

        self.qr_pick_ctrl_client.wait_for_result()
        if (self.qr_pick_ctrl_client.get_result()):
            print ("action succeed")                
            self.issucceed = True
        else:
            print ("action failed")
            self.issucceed = False

    def do_qrtable1plantapproach(self, arg):
        'place the plant to the table1 position w.r.t. qr marker'

        self.issucceed = False

        goal = spot_kinova_msgs.msg.QRPickGoal

        # goal.topic_name = "aruco_markers_pos4"
        goal.qr_pose = self.qr4_pose
        
        goal.target_pose = Pose()                
        goal.approach_offset = 0.1

        goal.target_pose.position.x    =                        self.table2_qr4_to_plant1_x  
        goal.target_pose.position.y    =                        self.table2_qr4_to_plant1_y
        goal.target_pose.position.z    =  self.gripper_length + 0.025
        
        if (self.gripper_change):
            goal.target_pose.orientation.x =  0.0
            goal.target_pose.orientation.y =  0.0
            goal.target_pose.orientation.z =  1.0
            goal.target_pose.orientation.w =  0.0        
        else:
            goal.target_pose.orientation.x =  0.0
            goal.target_pose.orientation.y =  0.0
            goal.target_pose.orientation.z =  0.0
            goal.target_pose.orientation.w =  1.0        

        goal.duration = 3.0        

        print ("action sent")
        self.qr_pick_ctrl_client.send_goal(goal) 

        self.qr_pick_ctrl_client.wait_for_result()
        if (self.qr_pick_ctrl_client.get_result()):
            print ("action succeed")                
            self.issucceed = True
        else:
            print ("action failed")
            self.issucceed = False 

    def do_qrplant2pick_fromtray(self, arg):
        'pick the plant2 w.r.t. qr marker'

        self.issucceed = False

        goal = spot_kinova_msgs.msg.QRPickGoal

        # goal.topic_name = "aruco_markers_pose5"
        goal.qr_pose = self.qr5_pose
        
        goal.target_pose = Pose()                
        goal.approach_offset = 0.1
        
        goal.target_pose.position.x    =                        self.tray_qr5_to_plant2_x + self.pick_x_offset
        goal.target_pose.position.y    =                        self.tray_qr5_to_plant2_y
        goal.target_pose.position.z    =  self.gripper_length + 0.015
        
        if (self.gripper_change):
            goal.target_pose.orientation.x =  0.0
            goal.target_pose.orientation.y =  0.0
            goal.target_pose.orientation.z =  1.0
            goal.target_pose.orientation.w =  0.0        
        else:
            goal.target_pose.orientation.x =  0.0
            goal.target_pose.orientation.y =  0.0
            goal.target_pose.orientation.z =  0.0
            goal.target_pose.orientation.w =  1.0        

        goal.duration = 3.0        

        print ("action sent")
        self.qr_pick_ctrl_client.send_goal(goal) 

        self.qr_pick_ctrl_client.wait_for_result()
        if (self.qr_pick_ctrl_client.get_result()):
            print ("action succeed")                
            self.issucceed = True
        else:
            print ("action failed")
            self.issucceed = False

    def do_qrtable2plantapproach(self, arg):
        'place the plant to the table2 position w.r.t. qr marker'

        self.issucceed = False

        goal = spot_kinova_msgs.msg.QRPickGoal

        # goal.topic_name = "aruco_markers_pos4"
        goal.qr_pose = self.qr4_pose
        
        goal.target_pose = Pose()                
        goal.approach_offset = 0.1 

        goal.target_pose.position.x    =                        self.table2_qr4_to_plant2_x   
        goal.target_pose.position.y    =                        self.table2_qr4_to_plant2_y 
        goal.target_pose.position.z    =  self.gripper_length + 0.025
        
        if (self.gripper_change):
            goal.target_pose.orientation.x =  0.0
            goal.target_pose.orientation.y =  0.0
            goal.target_pose.orientation.z =  1.0
            goal.target_pose.orientation.w =  0.0        
        else:
            goal.target_pose.orientation.x =  0.0
            goal.target_pose.orientation.y =  0.0
            goal.target_pose.orientation.z =  0.0
            goal.target_pose.orientation.w =  1.0        

        goal.duration = 3.0        

        print ("action sent")
        self.qr_pick_ctrl_client.send_goal(goal) 

        self.qr_pick_ctrl_client.wait_for_result()
        if (self.qr_pick_ctrl_client.get_result()):
            print ("action succeed")                
            self.issucceed = True
        else:
            print ("action failed")
            self.issucceed = False 
    

    ####################################################### jog ###########################################################                    
    def do_armup(self, arg):
        if (int(arg) <= 200):
            goal = spot_kinova_msgs.msg.SE3ArrayGoal

            self.issucceed = False

            goal.relative = True
            goal.target_poses = PoseArray()
            pose_se = Pose()
            pose_se.position.x = 0.0
            pose_se.position.y = 0.0
            pose_se.position.z = int(arg) * 0.001
            pose_se.orientation.x =  0.0
            pose_se.orientation.y = 0.0
            pose_se.orientation.z = 0.0
            pose_se.orientation.w = 1.0
            goal.target_poses.poses.append(pose_se)
            
            goal.durations = Float32MultiArray()
            goal.durations.data.append(2.0)        
            
            print ("action sent")        
            self.se3_array_ctrl_client.send_goal(goal)

            self.se3_array_ctrl_client.wait_for_result()
            if (self.se3_array_ctrl_client.get_result()):
                print ("action succeed")
                self.issucceed = True
            else:
                print ("action failed")
                self.issucceed = False
        else:
            print ("not exceed 200")

    def do_armdown(self, arg):
        if (int(arg) <= 200):
            goal = spot_kinova_msgs.msg.SE3ArrayGoal

            self.issucceed = False

            goal.relative = True
            goal.target_poses = PoseArray()
            pose_se = Pose()
            pose_se.position.x = 0.0
            pose_se.position.y = 0.0
            pose_se.position.z = int(arg) * 0.001 * -1.0
            pose_se.orientation.x =  0.0
            pose_se.orientation.y = 0.0
            pose_se.orientation.z = 0.0
            pose_se.orientation.w = 1.0
            goal.target_poses.poses.append(pose_se)
            
            goal.durations = Float32MultiArray()
            goal.durations.data.append(2.0)        
            
            print ("action sent")        
            self.se3_array_ctrl_client.send_goal(goal)

            self.se3_array_ctrl_client.wait_for_result()
            if (self.se3_array_ctrl_client.get_result()):
                print ("action succeed")
                self.issucceed = True
            else:
                print ("action failed")
                self.issucceed = False
        else:
            print ("not exceed 200")

    def do_armleft(self, arg):
        if (int(arg) <= 200):
            goal = spot_kinova_msgs.msg.SE3ArrayGoal

            self.issucceed = False

            goal.relative = True
            goal.target_poses = PoseArray()
            pose_se = Pose()
            pose_se.position.x = 0.0
            pose_se.position.y = int(arg) * 0.001
            pose_se.position.z = 0.0
            pose_se.orientation.x =  0.0
            pose_se.orientation.y = 0.0
            pose_se.orientation.z = 0.0
            pose_se.orientation.w = 1.0
            goal.target_poses.poses.append(pose_se)
            
            goal.durations = Float32MultiArray()
            goal.durations.data.append(2.0)        
            
            print ("action sent")        
            self.se3_array_ctrl_client.send_goal(goal)

            self.se3_array_ctrl_client.wait_for_result()
            if (self.se3_array_ctrl_client.get_result()):
                print ("action succeed")
                self.issucceed = True
            else:
                print ("action failed")
                self.issucceed = False
        else:
            print ("not exceed 200")

    def do_armright(self, arg):
        if (int(arg) <= 200):
            goal = spot_kinova_msgs.msg.SE3ArrayGoal

            self.issucceed = False

            goal.relative = True
            goal.target_poses = PoseArray()
            pose_se = Pose()
            pose_se.position.x = 0.0
            pose_se.position.y = int(arg) * 0.001  * -1.0
            pose_se.position.z = 0.0
            pose_se.orientation.x =  0.0
            pose_se.orientation.y = 0.0
            pose_se.orientation.z = 0.0
            pose_se.orientation.w = 1.0
            goal.target_poses.poses.append(pose_se)
            
            goal.durations = Float32MultiArray()
            goal.durations.data.append(2.0)        
            
            print ("action sent")        
            self.se3_array_ctrl_client.send_goal(goal)

            self.se3_array_ctrl_client.wait_for_result()
            if (self.se3_array_ctrl_client.get_result()):
                print ("action succeed")
                self.issucceed = True
            else:
                print ("action failed")
                self.issucceed = False
        else:
            print ("not exceed 200")

    def do_armforward(self, arg):
        if (int(arg) <= 200):
            goal = spot_kinova_msgs.msg.SE3ArrayGoal

            self.issucceed = False

            goal.relative = True
            goal.target_poses = PoseArray()
            pose_se = Pose()
            pose_se.position.x = int(arg) * 0.001
            pose_se.position.y = 0.0
            pose_se.position.z = 0.0
            pose_se.orientation.x =  0.0
            pose_se.orientation.y = 0.0
            pose_se.orientation.z = 0.0
            pose_se.orientation.w = 1.0
            goal.target_poses.poses.append(pose_se)
            
            goal.durations = Float32MultiArray()
            goal.durations.data.append(2.0)        
            
            print ("action sent")        
            self.se3_array_ctrl_client.send_goal(goal)

            self.se3_array_ctrl_client.wait_for_result()
            if (self.se3_array_ctrl_client.get_result()):
                print ("action succeed")
                self.issucceed = True
            else:
                print ("action failed")
                self.issucceed = False
        else:
            print ("not exceed 200")

    def do_armback(self, arg):
        if (int(arg) <= 200):
            goal = spot_kinova_msgs.msg.SE3ArrayGoal

            self.issucceed = False

            goal.relative = True
            goal.target_poses = PoseArray()
            pose_se = Pose()
            pose_se.position.x = int(arg) * 0.001 * -1.0
            pose_se.position.y = 0.0
            pose_se.position.z = 0.0
            pose_se.orientation.x =  0.0
            pose_se.orientation.y = 0.0
            pose_se.orientation.z = 0.0
            pose_se.orientation.w = 1.0
            goal.target_poses.poses.append(pose_se)
            
            goal.durations = Float32MultiArray()
            goal.durations.data.append(2.0)        
            
            print ("action sent")        
            self.se3_array_ctrl_client.send_goal(goal)

            self.se3_array_ctrl_client.wait_for_result()
            if (self.se3_array_ctrl_client.get_result()):
                print ("action succeed")
                self.issucceed = True
            else:
                print ("action failed")
                self.issucceed = False 
        else:
            print ("not exceed 200")

    #########################################################################################################################
    ########################################## kinova predefined motion #####################################################
    #########################################################################################################################
    def do_preready(self, arg):
        'Go to the preready position using predefined posture ctrl'
        goal = spot_kinova_msgs.msg.PredefinedPostureGoal
        goal.posture_name = "Ready"

        print ("action sent")
        self.predefined_posture_ctrl_client.send_goal(goal)
        self.predefined_posture_ctrl_client.wait_for_result()
        if (self.predefined_posture_ctrl_client.get_result()):
            print ("action succeed")
        else:
            print ("action failed")

    def do_preapproach(self, arg):
        'Go to the preapproach position using predefined posture ctrl'
        goal = spot_kinova_msgs.msg.PredefinedPostureGoal
        goal.posture_name = "Approach"

        print ("action sent")
        self.predefined_posture_ctrl_client.send_goal(goal)
        self.predefined_posture_ctrl_client.wait_for_result()
        if (self.predefined_posture_ctrl_client.get_result()):
            print ("action succeed")
        else:
            print ("action failed")

    def do_prehome(self, arg):
        'Go to the home position using predefined posture ctrl'
        goal = spot_kinova_msgs.msg.PredefinedPostureGoal
        goal.posture_name = "NewHome"

        print ("action sent")
        self.predefined_posture_ctrl_client.send_goal(goal)
        self.predefined_posture_ctrl_client.wait_for_result()
        if (self.predefined_posture_ctrl_client.get_result()):
            print ("action succeed")
        else:
            print ("action failed")

    def do_prefold(self, arg):
        'Go to the prefold position using predefined posture ctrl'
        goal = spot_kinova_msgs.msg.PredefinedPostureGoal
        goal.posture_name = "NewTurnOff"

        print ("action sent")
        self.predefined_posture_ctrl_client.send_goal(goal)
        self.predefined_posture_ctrl_client.wait_for_result()
        if (self.predefined_posture_ctrl_client.get_result()):
            print ("action succeed")
        else:
            print ("action failed")

    def do_premechpose(self, arg):
        'Go to the mechpose position using predefined posture ctrl'
        goal = spot_kinova_msgs.msg.PredefinedPostureGoal
        goal.posture_name = "mechpose"

        print ("action sent")
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

        print ("action sent")
        self.joint_ctrl_client.send_goal(goal)
        self.joint_ctrl_client.wait_for_result()
        if (self.joint_ctrl_client.get_result()):
            print ("action succeed")
        else:
            print ("action failed")

    #########################################################################################################################
    ################################################## gripper ##############################################################
    #########################################################################################################################

    def do_open(self, arg):
        'Open Gripper'

        self.issucceed = False

        goal = spot_kinova_msgs.msg.GripperGoal
        # goal.open = True
        goal.position = 0.0

        print ("action sent")
        self.gripper_ctrl_client.send_goal(goal)
        self.gripper_ctrl_client.wait_for_result()

        if (self.gripper_ctrl_client.get_result()):
            print ("action succeed")
            self.issucceed = True
        else:
            print ("action failed")
            self.issucceed = False
        
    def do_close(self, arg):
        'Close Gripper'

        self.issucceed = False

        goal = spot_kinova_msgs.msg.GripperGoal
        # goal.open = False
        goal.position = 1.0

        print ("action sent")
        self.gripper_ctrl_client.send_goal(goal)
        self.gripper_ctrl_client.wait_for_result()

        if (self.gripper_ctrl_client.get_result()):
            print ("action succeed")
            self.issucceed = True
        else:
            print ("action failed")
            self.issucceed = False

    def do_position(self, arg):
        'position of Gripper'

        self.issucceed = False

        goal = spot_kinova_msgs.msg.GripperGoal
        gripper_position = int(arg)        
        
        if (gripper_position >= 0 and gripper_position <= 100):        
            goal.position = gripper_position/100.0

            print ("action sent", goal.position)
            self.gripper_ctrl_client.send_goal(goal)
            self.gripper_ctrl_client.wait_for_result()

            if (self.gripper_ctrl_client.get_result()):
                print ("action succeed")
                self.issucceed = True
            else:
                print ("action failed")
                self.issucceed = False
        else: 
            print ("gripper position only valid between 0.0 - 1.0. try again")   

    #########################################################################################################################
    ################################################## sequneces ############################################################
    #########################################################################################################################
    def do_test_full_mech(self, arg):
        self.do_walk_to_goal('mech')
        self.do_test_mech_pnp(6)

    def do_test_mech_pnp(self, arg):    
        if (self.issucceed):
            # self.do_arm_mechpose(arg)              #fold arm for mech recognition
            self.do_premechpose(arg)
        if (self.issucceed):
            self.do_mechcall(arg)                  #designated number (arg) object recognition
        if (self.issucceed):
            self.do_position(0)                    #place the object
        if (self.issucceed):
            self.do_preready(arg)                  #designated number (arg) object recognition

        if(self.isgoodpose):
            if (self.issucceed):
                time.sleep(5)                      #wait before qr recogintion
                self.do_qr1save(arg)               #save qr1 pose
            if (self.issucceed):
                self.do_position(50)               #open gripper for object picking        
            if (self.issucceed):
                self.do_mechqrpick(arg)            #pick the object w.r.t marker 5 (qr1)    
            if (self.issucceed):
                self.do_position(95)               #grasp the object
            if (self.issucceed):
                self.do_pickup(arg)                #pick up the object        
            if (self.issucceed):
                self.do_gotobox(arg)               #go to box to place object
            if (self.issucceed):
                self.do_position(50)               #place the object
            if (self.issucceed):
                self.do_preready(arg)              #go to preready
            print ("pick and place with mechmind sequence test succeed")  
        else:
            print ("mech pnp is finished")  

    def do_test_box_pick(self, arg):    
        if (self.issucceed):
            self.do_prefold(arg)                   #go to prefold
        if (self.issucceed):
            self.do_preapproach(arg)               #go to box qr position
        if (self.issucceed):
            time.sleep(5)                          #wait before qr recogintion
            self.do_qr2save(arg)                   #save qr2 pose
        if (self.issucceed):
            self.do_position(70)                   #oepn gripper to approach to the box
        if (self.issucceed):
            self.do_qrboxpick(arg)                 #pick the box w.r.t marker 41
        if (self.issucceed):
            self.do_position(95)                   #grasp the box              
        if (self.issucceed):
            self.do_pickup(arg)                    #pick up the object
        if (self.issucceed):
            self.do_prefold(arg)                   #go to prefold
        print ("pick the box sequence test succeed") 
        
    def do_test_box_place(self, arg):    
        if (self.issucceed):
            self.do_prefold(arg)               #go to tray palce location
        if (self.issucceed):
            self.do_goto_boxplace(arg)            #go to tray palce location                        
        # if (self.issucceed):
        #     time.sleep(5)                          #wait before qr recogintion
        #     self.do_qr6_spot_save(arg)                   #save qr5 pose
        # if (self.issucceed):
        #     self.do_qrboxplace(arg)
        if (self.issucceed):
            self.do_armdown(50)               #go to tray palce location
        if (self.issucceed):
            self.do_position(70)                    #place the tray                      
        if (self.issucceed):
            self.do_armback(100)               #go to tray palce location
        if (self.issucceed):
            self.do_prefold(arg)                   #go to prefold
        print ("pick the tray sequence test succeed")   

    def do_test_tray_pick(self, arg):    
        if (self.issucceed):
            self.do_preapproach(arg)               #go to preapproach 
        if (self.issucceed):
            self.do_position(0)                    #open the gripper for qr recognition
        if (self.issucceed):
            self.do_armdown(100)                   #down to see the qr5 
        if (self.issucceed):
            time.sleep(5)                          #wait before qr recogintion
            self.do_qr5save(arg)                   #save qr5 pose
        if (self.issucceed):
            self.do_position(70)                   #half close the gripper before approach to the tray
        if (self.issucceed):
            self.do_qrtraypick(arg)                #approach to the tray w.r.t marker 13
        if (self.issucceed):
            self.do_position(99)                   #grasp the tray              
        if (self.issucceed):
            self.do_armup(100)                     #pick up the object        
        if (self.issucceed):
            self.do_prefold(arg)                   #go to prefold
        print ("pick the tray sequence test succeed") 

    def do_test_tray_pick_changed_gripper(self, arg):    
        if (self.issucceed):
            self.do_preready(arg)               #go to preapproach 
        if (self.issucceed):
            self.do_position(0)                    #open the gripper for qr recognition        
        if (self.issucceed):
            time.sleep(5)                          #wait before qr recogintion
            self.do_qr5save(arg)                   #save qr5 pose
        if (self.issucceed):
            self.do_position(70)                   #half close the gripper before approach to the tray
        if (self.issucceed):
            self.do_qrtraypick_changed_gripper(arg)                #approach to the tray w.r.t marker 13
        if (self.issucceed):
            self.do_armforward(70)                #approach to the tray w.r.t marker 13
        if (self.issucceed):
            self.do_position(99)                   #grasp the tray              
        if (self.issucceed):
            self.do_armup(200)                     #pick up the object        
        if (self.issucceed):
            self.do_preready(arg)                   #go to prefold
        print ("pick the tray sequence test succeed")

    def do_test_tray_place(self, arg):    
        if (self.issucceed):
            self.do_preapproach(arg)               #go to tray palce location
        if (self.issucceed):
            self.do_goto_trayplace(arg)            #go to tray palce location                        
        # if (self.issucceed):
        #     time.sleep(5)                          #wait before qr recogintion
        #     self.do_qr4_spot_save(arg)                   #save qr5 pose
        # if (self.issucceed):
        #     self.do_qrtrayplace(arg)
        if (self.issucceed):
            self.do_armdown(50)               #go to tray palce location
        if (self.issucceed):
            self.do_position(70)                    #place the tray              
        if (self.issucceed):
            self.do_armdown(10)               #go to tray palce location
        if (self.issucceed):
            self.do_armback(100)               #go to tray palce location
        if (self.issucceed):
            self.do_prefold(arg)                   #go to prefold
        print ("pick the tray sequence test succeed")              

    def do_test_plant1_table_to_tray(self, arg):    
        if (self.issucceed):
            self.do_preready(arg)                  #go to preready to see tray qr (qr5)
        if (self.issucceed):
            self.do_position(0)                    #open the gripper for qr recognition
        if (self.issucceed):
            time.sleep(3)                          #wait before qr recoginition
            self.do_qr5save(arg)                   #save qr5 pose           
        if (self.issucceed):
            self.do_goto_tableqr(arg)                   #go left to see table#2 qr (qr4)                    
        if (self.issucceed):
            time.sleep(3)                          #wait before qr recognition
            self.do_qr3save(arg)                   #save qr5 pose           
        if (self.issucceed):
            self.do_position(40)                   #half close gripper before approching to the plant
        if (self.issucceed):
            self.do_qrplant1pick_fromtable1(arg)   #approach to the plant1 w.r.t qr5        
        if (self.issucceed):
            self.do_armforward(-int(self.pick_x_offset*1000))                 #approach to the plant1 w.r.t qr5        
        if (self.issucceed):
            self.do_position(70)                   #grasp the plant                      
        if (self.issucceed):
            self.do_armup(100)                     #pick up the plant
        if (self.issucceed):
            self.do_qrtray1plantapproach(arg)      #approach to the tray plant2 position
        if (self.issucceed):
            self.do_position(30)                   #place the plant
        if (self.issucceed):
            self.do_preready(arg)                  #go to preready
        print ("pick the plant sequence test succeed")

    def do_test_plant2_table_to_tray(self, arg):    
        if (self.issucceed):
            self.do_preready(arg)                  #go to preready to see tray qr (qr5)
        if (self.issucceed):
            self.do_position(0)                    #open the gripper for qr recognition
        if (self.issucceed):
            time.sleep(3)                          #wait before qr recoginition
            self.do_qr5save(arg)                   #save qr5 pose           
        if (self.issucceed):
            self.do_goto_tableqr(arg)                   #go left to see table#2 qr (qr4)            
        if (self.issucceed):
            time.sleep(3)                          #wait before qr recognition
            self.do_qr3save(arg)                   #save qr5 pose           
        if (self.issucceed):
            self.do_position(40)                   #half close gripper before approching to the plant
        if (self.issucceed):
            self.do_qrplant2pick_fromtable1(arg)   #approach to the plant2 w.r.t qr5        
        if (self.issucceed):
            self.do_armforward(-int(self.pick_x_offset*1000))                 #approach to the plant1 w.r.t qr5
        if (self.issucceed):
            self.do_position(70)                   #grasp the plant                      
        if (self.issucceed):
            self.do_armup(200)                     #pick up the plant
        if (self.issucceed):
            self.do_qrtray2plantapproach(arg)      #approach to the tray plant2 position
        if (self.issucceed):
            self.do_position(30)                   #place the plant
        if (self.issucceed):
            self.do_preready(arg)                  #go to preready
        print ("pick the plant sequence test succeed") 

    def do_test_plant1_tray_to_table(self, arg):    
        if (self.issucceed):
            self.do_preready(arg)                  #go to preready to see tray qr (qr5)
        if (self.issucceed):
            self.do_position(0)                    #open the gripper for qr recognition
        if (self.issucceed):
            time.sleep(3)                          #wait before qr recoginition
            self.do_qr5save(arg)                   #save qr5 pose           
        if (self.issucceed):
            self.do_goto_tableqr(arg)                   #go left to see table#2 qr (qr4)            
        if (self.issucceed):
            time.sleep(3)                          #wait before qr recognition
            self.do_qr4save(arg)                   #save qr5 pose           
        if (self.issucceed):
            self.do_position(40)                   #half close gripper before approching to the plant
        if (self.issucceed):
            self.do_qrplant1pick_fromtray(arg)     #approach to the plant1 w.r.t qr5        
        if (self.issucceed):
                self.do_armforward(-int(self.pick_x_offset*1000))                 #approach to the plant1 w.r.t qr5
        if (self.issucceed):
            self.do_position(70)                   #grasp the plant                      
        if (self.issucceed):
            self.do_armup(100)                     #pick up the plant
        if (self.issucceed):
            self.do_qrtable1plantapproach(arg)     #approach to the table#2 plant2 position
        if (self.issucceed):
            self.do_position(30)                   #place the plant
        if (self.issucceed):
            self.do_preready(arg)                  #go to preready
        print ("pick the plant sequence test succeed")

    def do_test_plant2_tray_to_table(self, arg):    
        if (self.issucceed):
            self.do_preready(arg)                  #go to preready to see tray qr (qr5)
        if (self.issucceed):
            self.do_position(0)                    #open the gripper for qr recognition
        if (self.issucceed):
            time.sleep(3)                          #wait before qr recoginition
            self.do_qr5save(arg)                   #save qr5 pose           
        if (self.issucceed):
            self.do_goto_tableqr(arg)                   #go left to see table#2 qr (qr4)            
        if (self.issucceed):
            time.sleep(3)                          #wait before qr recognition
            self.do_qr4save(arg)                   #save qr5 pose           
        if (self.issucceed):
            self.do_position(40)                   #half close gripper before approching to the plant
        if (self.issucceed):
            self.do_qrplant2pick_fromtray(arg)     #approach to the plant2 w.r.t qr5        
        if (self.issucceed):
                self.do_armforward(-int(self.pick_x_offset*1000))                 #approach to the plant1 w.r.t qr5
        if (self.issucceed):
            self.do_position(70)                   #grasp the plant                      
        if (self.issucceed):
            self.do_armup(100)                     #pick up the plant
        if (self.issucceed):
            self.do_qrtable2plantapproach(arg)     #approach to the table#2 plant2 position
        if (self.issucceed):
            self.do_position(30)                   #place the plant
        if (self.issucceed):
            self.do_preready(arg)                  #go to preready
        print ("pick the plant sequence test succeed")


    #########################################################################################################################
    ############################################# spot primitives ###########################################################
    #########################################################################################################################
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

        print ("action sent")
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

        print ("action sent")
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

        print ("action sent")
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

        print ("action sent")
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

        print ("action sent")
        self.wholebody_ctrl_client.send_goal(goal)
        self.wholebody_ctrl_client.wait_for_result()
        if (self.wholebody_ctrl_client.get_result()):
            print ("action succeed")
        else:
            print ("action failed")

    def do_walk_onestep(self, arg):
        goal = spot_kinova_msgs.msg.WalkGoal

        goal.target_pose = Pose()
        goal.target_pose.position.x = 0.4 #one step!
        goal.target_pose.position.y = 0.0
        goal.target_pose.position.z = 0.0

        goal.target_pose.orientation.x = 0.0
        goal.target_pose.orientation.y = 0.0
        goal.target_pose.orientation.z = 0.0
        goal.target_pose.orientation.w = 1.0


        goal.relative = True

        print ("action sent")
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

        print ("action sent")
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

            print ("action sent")
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

            #     print ("action sent")
            #     self.wholebody_ctrl_client.send_goal(goal)
            #     self.wholebody_ctrl_client.wait_for_result()
            #     if (self.wholebody_ctrl_client.get_result()):
            #         time.sleep(1)
            #         self.do_bodyflat(arg)

    def do_qrwalk(self, arg):
        'Walk toward the qr position'
        goal = spot_kinova_msgs.msg.QRGoal
        goal.topic_name = "aruco_markers_pose1"

        print ("action sent")
        self.qr_walk_ctrl_client.send_goal(goal)
        self.qr_walk_ctrl_client.wait_for_result()

    def do_start_recording(self, arg):
        'Start Recording'

        rospy.wait_for_service('/spot/start_recording')
        start_recording_service = rospy.ServiceProxy('/spot/start_recording', Trigger)
        resp = start_recording_service()

        print(resp)

    def do_stop_recording(self, arg):
        'Stop Recording'

        rospy.wait_for_service('/spot/stop_recording')
        stop_recording_service = rospy.ServiceProxy('/spot/stop_recording', Trigger)
        resp = stop_recording_service()

        print(resp)

    def do_clear_graph(self, arg):
        'Clear Graph'

        rospy.wait_for_service('/spot/clear_graph')
        clear_graph_service = rospy.ServiceProxy('/spot/clear_graph', Trigger)
        resp = clear_graph_service()

        print(resp)

    def do_download_graph(self, arg):
        'Download Graph'

        make_dir(self.map_root)

        prefix = 'kimm_map_'
        map_dirs = os.listdir(self.map_root)
        num = 0
        if len(map_dirs) > 0:
            map_dirs.sort()
            num = int(map_dirs[-1].split('_')[-1])+1

        save_path = os.path.join(self.map_root,prefix+f'{num:06d}')
        # make_dir(save_path)
        # print(save_path)

        rospy.wait_for_service('/spot/download_graph')
        download_graph_service = rospy.ServiceProxy('/spot/download_graph', DownloadGraph)

        resp = download_graph_service(save_path)
        print(resp)

    def do_upload_graph(self, arg):
        'Upload Graph'

        prefix = 'kimm_map_'
        num = int(arg)

        map_path = os.path.join(self.map_root,prefix+f'{num:06d}')

        if os.path.exists(map_path):
            rospy.wait_for_service('/spot/upload_graph')
            upload_graph_service = rospy.ServiceProxy('/spot/upload_graph', UploadGraph)
            resp = upload_graph_service(map_path)
            print(resp)
        else:
            print('map path does not exist')

    def do_list_graph(self, arg):
        'List Graph'

        rospy.wait_for_service('/spot/list_graph')
        list_graph_service = rospy.ServiceProxy('/spot/list_graph', ListGraph)
        resp = list_graph_service() #resp.waypoint_ids[0]
        self.waypoint_ids = resp.waypoint_ids

        print(resp)

    def do_set_localization_fiducial(self, arg):
        'Set localization fiducial'

        rospy.wait_for_service('/spot/set_localization_fiducial')
        set_localization_fiducial_service = rospy.ServiceProxy('/spot/set_localization_fiducial', SetLocalizationFiducial)
        resp = set_localization_fiducial_service()

        print(resp)

    def do_navigate_to(self, arg):
        'Navigate to arg'
        num = int(arg)

        goal = spot_msgs.msg.NavigateToActionGoal
        goal.id_navigate_to = self.waypoint_ids[num]
        print(goal.id_navigate_to)

        print ("action sent")
        self.navigate_to_client.send_goal(goal)
        self.navigate_to_client.wait_for_result()

        if (self.navigate_to_client.get_result()):
            print ("action succeed")
        else:
            print ("action failed")   

    def do_quit(self, arg):
        return True

if __name__ == '__main__':
    ControlSuiteShell().cmdloop()
