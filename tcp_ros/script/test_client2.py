#!/usr/bin/env python3
import time
from math import pi
import numpy as np

import rospy
from tcp_ros.msg import Trigger, GetResult, Feedback
from geometry_msgs.msg import PoseArray, Pose, Quaternion
from std_msgs.msg import String
from tf.transformations import *

target_label = 0

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

def tf_to_pose_msg(mat):
    t = mat[:3, 3]
    q = quaternion_from_matrix(mat)

    pose_msg = Pose()
    pose_msg.position.x = t[0]
    pose_msg.position.y = t[1]
    pose_msg.position.z = t[2]

    pose_msg.orientation.x = q[0]
    pose_msg.orientation.y = q[1]
    pose_msg.orientation.z = q[2]
    pose_msg.orientation.w = q[3]

    return pose_msg

def tf_from_pose_msg(pose_msg):
    q = [pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w]
    t = [pose_msg.position.x, pose_msg.position.y, pose_msg.position.z]
    T_mat = quaternion_matrix(q)
    T_mat[:3, 3] = t

    return T_mat

def tf_from_pose(translation, quaternion):
    T_mat = quaternion_matrix(quaternion)
    T_mat[:3, 3] = translation

    return T_mat

def pub_trigger_msg(pub_trigger):   
    msg = Trigger()
    msg.project = '1'
    msg.request = '20'
    
    pub_trigger.publish(msg)
    
    print('published trigger')
    
def pub_get_result_msg(pub_getresult):
    msg = GetResult()
    msg.project = '1'
    
    pub_getresult.publish(msg)    
    
    print('published getResult msg')
    
def feedback_msg_callback(data):
    print('get pose data from the mechmind')
    # print(len(data.position))
    # print(data.position[0])
    
    pose_array_msgs = PoseArray()
    pose_array_msgs.header = data.header

    marker_pose = Pose()
    # marker_pose.position.x = -0.05620725452899933
    # marker_pose.position.y = -0.09847493469715118
    # marker_pose.position.z = 1.0938366651535034
    # marker_pose.orientation.x = -0.666976
    # marker_pose.orientation.y = -0.0173173
    # marker_pose.orientation.z = 0.744240
    # marker_pose.orientation.w = 0.0308013677

    #jspark 0616
    #marker_pose.position.x =     0.01995144411921501
    #marker_pose.position.y =    -0.08609140664339066
    #marker_pose.position.z =     1.082968831062317
    #marker_pose.orientation.x =  0.7140572641951235
    #marker_pose.orientation.y =  0.6984106214406419
    #marker_pose.orientation.z =  0.02853158484453095
    #marker_pose.orientation.w = -0.039125132272379196

    #jspark 0715
    #marker_pose.position.x =     0.01380316
    #marker_pose.position.y =    -0.13231481
    #marker_pose.position.z =     1.06141376
    #marker_pose.orientation.x =  0.71799383
    #marker_pose.orientation.y =  0.69481206
    #marker_pose.orientation.z =  0.03652184
    #marker_pose.orientation.w = -0.01967741

    #jspark 0719
    #marker_pose.position.x =     0.01302440
    #marker_pose.position.y =    -0.13245843  
    #marker_pose.position.z =     1.05794358
    #marker_pose.orientation.x =  0.71889673
    #marker_pose.orientation.y =  0.69393109
    #marker_pose.orientation.z =  0.03956003
    #marker_pose.orientation.w = -0.00906237

    #jspark 0727 - testbed was changed
    marker_pose.position.x =    -0.009799563325941563
    marker_pose.position.y =    -0.05994241312146187
    marker_pose.position.z =     1.0877445936203003
    marker_pose.orientation.x =  0.6993550581616976
    marker_pose.orientation.y =  0.7130807433084043
    marker_pose.orientation.z =  0.037050852237289704
    marker_pose.orientation.w = -0.03233559176623464

    pose_array_msgs.poses.append(marker_pose)

    T_cm = tf_from_pose_msg(marker_pose)
    T_mc = inverse_matrix(T_cm)
    
    global target_label
    ispub = False
    for pose in data.position:                
        t_obj = (pose.linear.x*1e-3, pose.linear.y*1e-3, pose.linear.z*1e-3)
        q_obj = quaternion_from_euler(pose.angular.x*pi/180.0, pose.angular.y*pi/180.0, pose.angular.z*pi/180.0)
    
        pose_msg = Pose()
        pose_msg.position.x = pose.linear.x*1e-3
        pose_msg.position.y = pose.linear.y*1e-3
        pose_msg.position.z = pose.linear.z*1e-3    
        pose_msg.orientation = Quaternion(q_obj[0], q_obj[1], q_obj[2], q_obj[3])

        T_co = tf_from_pose_msg(pose_msg)                        
        T_mo = np.matmul(T_mc, T_co)
        pub_msg = tf_to_pose_msg(T_mo)
            
        pose_array_msgs.poses.append(pub_msg)
        # pose_array_msgs.poses.append(pose_msg)

        if ispub == False and pose.label == target_label:                                 
            pub_obj_pose.publish(pub_msg)   
            ispub = True
            print(bcolors.OKBLUE + "target object is recognized!" + bcolors.ENDC)            
            print(bcolors.OKBLUE + str(target_label) + bcolors.ENDC)            
            print(bcolors.OKBLUE + str(pose.label) + bcolors.ENDC)                        
    
    if ispub == False:
        print(bcolors.OKBLUE + "there is no target object." + bcolors.ENDC)                    
        # pub_obj_pose.publish(pub_msg)        
    
    pub_obj_poses.publish(pose_array_msgs)       

def call_from_robot(msg):
    global target_label

    time.sleep(1)

    if len(msg.data) > 9: #mech_call
        target_label = int(msg.data[10:])
    else:
        target_label = 1

    pub_trigger_msg(pub_trigger)

    time.sleep(1)
    pub_get_result_msg(pub_getresult)

    print(bcolors.OKBLUE + "target label is  " + str(target_label) + bcolors.ENDC)            

    time.sleep(1)

if __name__ == '__main__':    
    rospy.init_node('object_pose_node')#, anonymous=True)
    pub_trigger = rospy.Publisher('/trigger', Trigger, queue_size=10)
    pub_getresult = rospy.Publisher('/getResult', GetResult, queue_size=10)
    
    pub_obj_poses = rospy.Publisher('/obj_poses', PoseArray, queue_size=10)

    pub_obj_pose = rospy.Publisher('mechmind_publisher/pose', Pose, queue_size=10)

    rospy.Subscriber('/mech_call', String, call_from_robot)
    rospy.Subscriber('/pose', Feedback, feedback_msg_callback)

    rospy.spin()


    
# rostopic pub /trigger tcp_ros/Trigger "project: '1'"
# rostopic pub /trigger tcp_ros/Trigger "request: '20'"
# rostopic pub /getResult tcp_ros/GetResult "project: '1'"
