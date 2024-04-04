#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker
import tf
from tf.transformations import *
import numpy as np
import tf2_ros
from geometry_msgs.msg import TransformStamped

def tf_from_pose_msg(pose_msg):
    q = [pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w]
    t = [pose_msg.position.x, pose_msg.position.y, pose_msg.position.z]
    T_mat = quaternion_matrix(q)
    T_mat[:3, 3] = t

    return T_mat

def tf_from_lookupTransform(rot, trans):
    q = rot
    t = trans
    T_mat = quaternion_matrix(q)
    T_mat[:3, 3] = t

    return T_mat

def talker():
    qr_pick_pub = rospy.Publisher('kimm_rcnn/cup_pose', Pose, queue_size=1)
    qr_visual_pub = rospy.Publisher('cup_marker_viz', Marker, queue_size=1)

    rospy.init_node('cup_marer_talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    
    ref_frame = "m_camera_color_optical_frame"
    qr_pose = []
    T_pos = []

    while not rospy.is_shutdown():
        if ref_frame == "world":
            qr_pose= Pose()
            qr_pose.position.x =   0.6
            qr_pose.position.y =  -0.5
            qr_pose.position.z =   0.7-0.6
            qr_pose.orientation.x = 0.714
            qr_pose.orientation.y = 0.041
            qr_pose.orientation.z =-0.070
            qr_pose.orientation.w = 0.695     
            T_pos = tf_from_pose_msg(qr_pose)          
        
        elif ref_frame == "m_camera_color_optical_frame":
            qr_pose= Pose()
            qr_pose.position.x =   0.01
            qr_pose.position.y =  -0.02
            qr_pose.position.z =   0.27
            qr_pose.orientation.x = 0.0
            qr_pose.orientation.y = 0.0
            qr_pose.orientation.z = 1.0
            qr_pose.orientation.w = 0.0
            T_pos = tf_from_pose_msg(qr_pose)     
            
        T_new = T_pos    
        q_new = quaternion_from_matrix(T_new)
        
        # pose
        qr_new= Pose()
        qr_new.position.x =  T_new[0,3]
        qr_new.position.y =  T_new[1,3]
        qr_new.position.z =  T_new[2,3]
        qr_new.orientation.x = q_new[0]
        qr_new.orientation.y = q_new[1]
        qr_new.orientation.z = q_new[2]
        qr_new.orientation.w = q_new[3]  
        
        # marker
        marker = Marker()
        marker.header.frame_id = ref_frame
        marker.id = 100
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.pose.position.x = qr_new.position.x
        marker.pose.position.y = qr_new.position.y
        marker.pose.position.z = qr_new.position.z
        marker.pose.orientation.x = qr_new.orientation.x
        marker.pose.orientation.y = qr_new.orientation.y
        marker.pose.orientation.z = qr_new.orientation.z
        marker.pose.orientation.w = qr_new.orientation.w

        marker.scale.x = .1
        marker.scale.y = .1
        marker.scale.z = .01
        marker.color.a = 1
        marker.color.r = 0.0
        marker.color.g = 0.9
        marker.color.b = 0.2  

        # tf
        br = tf2_ros.TransformBroadcaster()
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = ref_frame
        transform.child_frame_id = "cup_marker"
        transform.transform.translation.x = qr_new.position.x
        transform.transform.translation.y = qr_new.position.y
        transform.transform.translation.z = qr_new.position.z
        transform.transform.rotation.x = qr_new.orientation.x
        transform.transform.rotation.y = qr_new.orientation.y
        transform.transform.rotation.z = qr_new.orientation.z
        transform.transform.rotation.w = qr_new.orientation.w
        
        qr_pick_pub.publish(qr_new)
        qr_visual_pub.publish(marker)
        br.sendTransform(transform)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass