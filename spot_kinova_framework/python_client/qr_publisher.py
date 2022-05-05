
#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker

def talker():
    qr_pick_pub = rospy.Publisher('qr_target_pose', Pose, queue_size=1)
    qr_visual_pub = rospy.Publisher('qr_marker_viz', Marker, queue_size=1)
    qr_goal_pub = rospy.Publisher('qr_goal_viz', Marker, queue_size=1)

    rospy.init_node('qr_talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        qr_pose= Pose()
        qr_pose.position.x = 0.9
        qr_pose.position.y = 0.1
        qr_pose.position.z = 0.5
        qr_pose.orientation.x =  0.0
        qr_pose.orientation.y = 0
        qr_pose.orientation.z = 0.7071068
        qr_pose.orientation.w = 0.7071068

        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.id = 100
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.pose.position.x = qr_pose.position.x
        marker.pose.position.y = qr_pose.position.y
        marker.pose.position.z = qr_pose.position.z
        marker.pose.orientation.x = qr_pose.orientation.x
        marker.pose.orientation.y = qr_pose.orientation.y
        marker.pose.orientation.z = qr_pose.orientation.z
        marker.pose.orientation.w = qr_pose.orientation.w

        marker.scale.x = .1
        marker.scale.y = .1
        marker.scale.z = .01
        marker.color.a = 1
        marker.color.r = 0.0
        marker.color.g = 0.9
        marker.color.b = 0.2

        qr_pick_pub.publish(qr_pose)
        qr_visual_pub.publish(marker)

        marker.pose.position.x = qr_pose.position.x - 0.1
        marker.pose.position.y = qr_pose.position.y + 0.2
        marker.pose.position.z = qr_pose.position.z
        marker.pose.orientation.x = qr_pose.orientation.x
        marker.pose.orientation.y = qr_pose.orientation.y
        marker.pose.orientation.z = qr_pose.orientation.z
        marker.pose.orientation.w = qr_pose.orientation.w

        marker.color.a = 1
        marker.color.r = 0.9
        marker.color.g = 0.0
        marker.color.b = 0.2
        
        qr_goal_pub.publish(marker)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
