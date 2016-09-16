#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import sys
import math
import numpy as np

import botpy

pub = rospy.Publisher('/ihmc_ros/localization/pelvis_odom_pose_correction', Odometry, queue_size=10)

def callback(m):
    rospy.loginfo(rospy.get_caller_id() + "I heard %f", m.pose.pose.position.x)

    orientation = [m.pose.pose.orientation.w, m.pose.pose.orientation.x, m.pose.pose.orientation.y, m.pose.pose.orientation.z]
    rpy = botpy.quat_to_euler(orientation)
    rpy[2] = rpy[2] + 1.0*math.pi/180.0
    orientation = botpy.euler_to_quat(rpy)  
    m.pose.pose.orientation.w = orientation[0]
    m.pose.pose.orientation.x = orientation[1]
    m.pose.pose.orientation.y = orientation[2]
    m.pose.pose.orientation.z = orientation[3]

    m.pose.pose.position.x = m.pose.pose.position.x - 0.10
    pub.publish(m)
    
    #msg = pose_t()
    #msg.utime = m.header.stamp.secs*1E6 + m.header.stamp.nsecs*1E-3
    #msg.pos = [m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z]
    #msg.orientation = 
    #lc.publish('POSE_BODY_CORRECTION_ROS', msg.encode() )

    print "Sent pose correction, exiting"
    rospy.signal_shutdown("Exiting")

def listener():
    rospy.init_node('send_fake_correction_ros', anonymous=True)
    rospy.Subscriber("/ihmc_ros/valkyrie/output/robot_pose", Odometry, callback)

    print "dfsdf"
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()