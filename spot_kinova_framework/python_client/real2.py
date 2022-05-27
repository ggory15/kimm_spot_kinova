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
import pinocchio as se3
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

        self.mech_call_pub = rospy.Publisher('/mech_call', String, queue_size=10)
        rospy.Subscriber('mechmind_publisher/pose', Pose, self.mechmind_pose_callback)
        self.mech_pose = Pose()
        self.iscallback_mech = False

    def mechmind_pose_callback(self, msg):        
        self.mech_pose = msg
        self.iscallback_mech = True
        print(self.mech_pose)        

        '''
        if (self.iscallback_mech):
            print ("pose received")
            self.iscallback_mech = False
        else:
            print ("not received")
        '''

    def do_mechcall(self, arg):
        mech_msg = String()        
        mech_msg.data = "mech_call_0" #request designated labeled object pose
        print ("pub mech_call")
        self.mech_call_pub.publish(mech_msg)

    def do_pickup(self, arg):
        goal = spot_kinova_msgs.msg.SE3ArrayGoal

        goal.relative = True
        goal.target_poses = PoseArray()
        pose_se = Pose()   
        pose_se.position.x = 0.0
        pose_se.position.y = 0.0
        pose_se.position.z = 0.15
        pose_se.orientation.x =  0.0
        pose_se.orientation.y = 0.0
        pose_se.orientation.z = 0.0
        pose_se.orientation.w = 1.0
        goal.target_poses.poses.append(pose_se)

        pose_se2 = Pose()
        pose_se2.position.x = 0.0
        pose_se2.position.y = -0.2
        pose_se2.position.z = 0.0
        pose_se2.orientation.x =  0.0
        pose_se2.orientation.y = 0.0
        pose_se2.orientation.z = 0.0
        pose_se2.orientation.w = 1.0
        goal.target_poses.poses.append(pose_se2)

        # pose_se2 = Pose()
        # pose_se2.position.x = 0.0
        # pose_se2.position.y = 0.0
        # pose_se2.position.z = -0.15
        # pose_se2.orientation.x =  0.0
        # pose_se2.orientation.y = 0.0
        # pose_se2.orientation.z = 0.0
        # pose_se2.orientation.w = 1.0
        # goal.target_poses.poses.append(pose_se2)

        # pose_se = Pose()   
        # pose_se.position.x = 0.0
        # pose_se.position.y = 0.0
        # pose_se.position.z = 0.1
        # pose_se.orientation.x =  0.0
        # pose_se.orientation.y = 0.0
        # pose_se.orientation.z = 0.0
        # pose_se.orientation.w = 1.0
        # goal.target_poses.poses.append(pose_se)

        # pose_se2 = Pose()
        # pose_se2.position.x = 0.1
        # pose_se2.position.y = 0.2
        # pose_se2.position.z = 0.0
        # pose_se2.orientation.x =  0.0
        # pose_se2.orientation.y = 0.0
        # pose_se2.orientation.z = 0.0
        # pose_se2.orientation.w = 1.0
        # goal.target_poses.poses.append(pose_se2)

        # pose_se2 = Pose()
        # pose_se2.position.x = 0.0
        # pose_se2.position.y = 0.0
        # pose_se2.position.z = -0.1
        # pose_se2.orientation.x =  0.0
        # pose_se2.orientation.y = 0.0
        # pose_se2.orientation.z = 0.0
        # pose_se2.orientation.w = 1.0
        # goal.target_poses.poses.append(pose_se2)

        
        goal.durations = Float32MultiArray()
        goal.durations.data.append(2.0)
        goal.durations.data.append(4.0)
        # goal.durations.data.append(3.0)
        # goal.durations.data.append(2.0)
        # goal.durations.data.append(3.0)
        # goal.durations.data.append(3.0)
        print ("action sent")

        self.se3_array_ctrl_client.send_goal(goal)
        
        self.se3_array_ctrl_client.wait_for_result()
        if (self.se3_array_ctrl_client.get_result()):
            print ("action succeed")
        else:
            print ("action failed")

    def do_qrpick(self, arg):
        goal = spot_kinova_msgs.msg.QRPickGoal

        goal.topic_name = "aruco_markers_pose1"
        goal.target_pose = Pose()

        #goal.target_pose.position.x = 0.1
        #goal.target_pose.position.y = 0.0
        #goal.target_pose.position.z = 0.15
        #goal.target_pose.orientation.x =  0.0
        #goal.target_pose.orientation.y = 0
        #goal.target_pose.orientation.z = 0.0
        #goal.target_pose.orientation.w = 1.0

        goal.target_pose.position.x    =  self.mech_pose.position.x
        goal.target_pose.position.y    =  self.mech_pose.position.y
        goal.target_pose.position.z    =  0.25 + self.mech_pose.position.z
        goal.target_pose.orientation.x =  self.mech_pose.orientation.x
        goal.target_pose.orientation.y =  self.mech_pose.orientation.y
        goal.target_pose.orientation.z =  self.mech_pose.orientation.z
        goal.target_pose.orientation.w =  self.mech_pose.orientation.w

        goal.duration = 3.0
        print ("action sent")

        self.qr_pick_ctrl_client.send_goal(goal)
        
        self.qr_pick_ctrl_client.wait_for_result()
        if (self.qr_pick_ctrl_client.get_result()):
            print ("action succeed")
        else:
            print ("action failed")
    def do_preready(self, arg):
        'Go to the home position using predefined posture ctrl'
        goal = spot_kinova_msgs.msg.PredefinedPostureGoal
        goal.posture_name = "Ready"

        print ("anction sent")
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
        goal.target_pose.position.x = 0.4
        goal.target_pose.position.y = 0.0
        goal.target_pose.position.z = 0.0

        goal.target_pose.orientation.x = 0.0
        goal.target_pose.orientation.y = 0.0
        goal.target_pose.orientation.z = 0.0
        goal.target_pose.orientation.w = 1.0


        goal.relative = True

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
        
        print ("anction sent")
        self.navigate_to_client.send_goal(goal)
        self.navigate_to_client.wait_for_result()

        if (self.navigate_to_client.get_result()):
            print ("action succeed")
        else:
            print ("action failed")

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
