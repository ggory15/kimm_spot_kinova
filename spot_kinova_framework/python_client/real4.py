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
        self.mech_call_pub = rospy.Publisher('/mech_call', String, queue_size=10)
        rospy.Subscriber('mechmind_publisher/pose', Pose, self.mechmind_pose_callback)
        
        self.qr1_flag = True
        self.qr2_flag = True
        self.qr3_flag = True
        self.qr4_flag = True
        self.qr5_flag = True
        rospy.Subscriber('aruco_markers_pose1', Pose, self.qr1_callback)
        rospy.Subscriber('aruco_markers_pose2', Pose, self.qr2_callback)
        rospy.Subscriber('aruco_markers_pose3', Pose, self.qr3_callback)
        rospy.Subscriber('aruco_markers_pose4', Pose, self.qr4_callback)
        rospy.Subscriber('aruco_markers_pose5', Pose, self.qr5_callback)                    

        self.gripper_length = 0.2      

    #########################################################################################################################
    ################################################ mechmind & qr###########################################################
    #########################################################################################################################    
    def do_mechcall(self, arg):
        'call the pose from the mechmind with designated object label'

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

        print ("pose received, do mechqrpick!")
        print(self.mech_pose)      
    
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
    
    #########################################################################################################################
    ############################################## kinova primitives ########################################################
    #########################################################################################################################                    
    
    ################################################## object pnp ###########################################################                    
    def do_arm_mechpose(self, arg):
        'Go to the up position for mechcall'

        self.issucceed = False

        goal = spot_kinova_msgs.msg.JointPostureGoal
        goal.duration = 5.0
        goal.target_joints = JointState()        
        goal.target_joints.position = np.array([0.0, -2.4, 3.14, -2.443, 0, 1.7, 1.57])

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

    def do_qrpick(self, arg):
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
    def do_goto_qrbox(self, arg):
        goal = spot_kinova_msgs.msg.SE3ArrayGoal

        self.issucceed = False

        goal.relative = True
        goal.target_poses = PoseArray()
        pose_se = Pose()
        pose_se.position.x = 0.0
        pose_se.position.y = -0.30
        pose_se.position.z = 0.0
        pose_se.orientation.x =  0.0
        pose_se.orientation.y = 0.0
        pose_se.orientation.z = 0.0
        pose_se.orientation.w = 1.0
        goal.target_poses.poses.append(pose_se)

        pose_se2 = Pose()
        pose_se2.position.x = 0.0
        pose_se2.position.y = 0.0
        pose_se2.position.z = -0.35
        pose_se2.orientation.x =  0.0
        pose_se2.orientation.y = 0.0
        pose_se2.orientation.z = 0.0
        pose_se2.orientation.w = 1.0
        goal.target_poses.poses.append(pose_se2)
        
        goal.durations = Float32MultiArray()
        goal.durations.data.append(3.0)        
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

    def do_qrboxpick(self, arg):
        'pick the box w.r.t. qr marker'

        self.issucceed = False

        goal = spot_kinova_msgs.msg.QRPickGoal

        # goal.topic_name = "aruco_markers_pose2"
        goal.qr_pose = self.qr2_pose
        
        goal.target_pose = Pose()                
        goal.approach_offset = 0.2 

        goal.target_pose.position.x    =  0.01                         #distance btw qr and grasp position in x direction w.r.t marker
        goal.target_pose.position.y    = -0.068                        #distance btw qr and box end in y direction w.r.t marker  
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

    ####################################################### jog ###########################################################                    
    def do_armup(self, arg):
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

    def do_armdown(self, arg):
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

    def do_armleft(self, arg):
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

    def do_armright(self, arg):
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

    def do_armforward(self, arg):
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

    def do_armback(self, arg):
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

    #########################################################################################################################
    ########################################## kinova predefined motion #####################################################
    #########################################################################################################################
    def do_preready(self, arg):
        'Go to the home position using predefined posture ctrl'
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
        'Go to the home position using predefined posture ctrl'
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
        'Go to the home position using predefined posture ctrl'
        goal = spot_kinova_msgs.msg.PredefinedPostureGoal
        goal.posture_name = "NewTurnOff"

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
    def do_test_mech_pnp(self, arg):    
        if (self.issucceed):
            self.do_arm_mechpose(arg)   #fold arm for mech recognition
        if (self.issucceed):
            self.do_mechcall(arg)  #designated number (arg) object recognition
        if (self.issucceed):
            self.do_preready(arg)  #designated number (arg) object recognition
        if (self.issucceed):
            self.do_qr1save(arg)   #open gripper for object picking        
        if (self.issucceed):
            self.do_position(50)   #open gripper for object picking        
        if (self.issucceed):
            self.do_mechqrpick(arg)    #pick the object w.r.t marker 5     
        if (self.issucceed):
            self.do_position(95)   #pick the object
        if (self.issucceed):
            self.do_pickup(arg)    #pick up the object        
        if (self.issucceed):
            self.do_gotobox(arg)   #go to box marker               
        if (self.issucceed):
            self.do_position(50)   #open gripper for box pikcing
        if (self.issucceed):
            self.do_preready(arg)  #go to preready
        print ("pick and place with mechmind sequence test succeed")  

    def do_test_box_pick(self, arg):    
        if (self.issucceed):
            self.do_prefold(arg)  #go to prefold
        if (self.issucceed):
            self.do_preapproach(arg)  #go to box qr position
        if (self.issucceed):
            self.do_qr2save(arg)  #go to box qr position
        if (self.issucceed):
            self.do_position(70)   #pick the box              
        if (self.issucceed):
            self.do_qrboxpick(arg) #pick the box w.r.t marker 41
        if (self.issucceed):
            self.do_position(95)   #pick the box              
        if (self.issucceed):
            self.do_pickup(arg)    #pick up the object
        if (self.issucceed):
            self.do_prefold(arg)  #go to preready
        print ("pick the box sequence test succeed")  

    def do_test(self, arg):    
        if (self.issucceed):
            self.do_preready(arg)  #ready
        if (self.issucceed):
            self.do_position(70)   #open gripper for object picking
        if (self.issucceed):
            self.do_qrpick(arg)    #pick the object w.r.t marker 5     
        if (self.issucceed):
            self.do_position(95)   #pick the object
        if (self.issucceed):
            self.do_pickup(arg)    #pick up the object
        if (self.issucceed):
            self.do_preready(arg)  #go to preready
        if (self.issucceed):
            self.do_gotobox(arg)   #go to box marker               
        if (self.issucceed):
            self.do_position(50)   #open gripper for box pikcing
        if (self.issucceed):
            self.do_qrboxpick(arg) #pick the box w.r.t marker 41
        if (self.issucceed):
            self.do_position(95)   #pick the box              
        if (self.issucceed):
            self.do_preready(arg)  #go to preready
        print ("pick and place sequence test succeed")

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
