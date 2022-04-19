#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from champ_msgs.msg import Pose as PoseLite
from geometry_msgs.msg import Pose as Pose
import tf
import numpy

rospy.init_node("Example")

pose_lite_publisher = rospy.Publisher('body_pose/raw', PoseLite, queue_size = 1)
pose_publisher = rospy.Publisher('body_pose', Pose, queue_size = 1)

body_pose_lite = PoseLite()
body_pose_lite.x = 0
body_pose_lite.y = 0
body_pose_lite.roll = 0.0 
body_pose_lite.pitch = 0 
body_pose_lite.yaw = 0 


r = rospy.Rate(10)
cnt = 0

while not rospy.is_shutdown():
    body_pose_lite.roll = 0.5 * numpy.sin(numpy.pi/60000. * cnt)
    pose_lite_publisher.publish(body_pose_lite)

    body_pose = Pose()
    body_pose.position.z = body_pose_lite.z

    quaternion = tf.transformations.quaternion_from_euler(body_pose_lite.roll, body_pose_lite.pitch, body_pose_lite.yaw)
    body_pose.orientation.x = quaternion[0]
    body_pose.orientation.y = quaternion[1]
    body_pose.orientation.z = quaternion[2]
    body_pose.orientation.w = quaternion[3]


    pose_lite_publisher.publish(body_pose_lite)
    pose_publisher.publish(body_pose)
    cnt = cnt +1
    