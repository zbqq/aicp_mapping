#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped

import numpy as np
import time
import sys


def talker():
    pub = rospy.Publisher("/interaction_marker/pose", PoseStamped, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 1hz

    pose = PoseStamped()
    try:
        pose.header.frame_id = sys.argv[1]
    except:
        pose.header.frame_id = "map_test"
    
    t = rospy.get_rostime() 
    pose.header.stamp.secs = t.secs
    pose.header.stamp.nsecs = t.nsecs

    pose.pose.position.x=0
    pose.pose.position.y=0
    pose.pose.position.z=0
    pose.pose.orientation.w=1
    pose.pose.orientation.x=0
    pose.pose.orientation.y=0
    pose.pose.orientation.z=0

    if (1==1): # rosbag play --clock -s 0 --pause [PATH_TO]/anymal_2018-10-28-15-56-41.bag
        pose.pose.position.x=11.0284013748
        pose.pose.position.y=0.316449940205
        pose.pose.position.z=-0.656497001648
        pose.pose.orientation.x=1.80515562533e-05
        pose.pose.orientation.y=9.31337854126e-05
        pose.pose.orientation.z=0.70231616497
        pose.pose.orientation.w=0.711905956268
  
    if (1==0): # rosbag play --clock --pause -s 150 [PATH_TO]/2018-08-13-fire-college/anymal_2018-08-13-15-26-49.bag
        pose.pose.position.x=8.05760669708
        pose.pose.position.y=2.24764990807
        pose.pose.position.z=-0.658026218414 
        pose.pose.orientation.x=2.56411240116e-05
        pose.pose.orientation.y=-2.81750290014e-05
        pose.pose.orientation.z=0.454391390085
        pose.pose.orientation.w=0.890812516212

    msg_str = "[Python] Publish base pose initial guess in frame: %s" % pose.header.frame_id 
    rospy.loginfo(msg_str)

    time.sleep(5)
    pub.publish(pose)
    time.sleep(5)
    pub.publish(pose)
    time.sleep(5)
    pub.publish(pose)
    time.sleep(5)
    pub.publish(pose)

if __name__ == '__main__':
    try:

        # print 'Syntax: rosrun aicp set_aicp_initial_guess.py [fixed_frame]'
        # print 'Number of arguments:', len(sys.argv), 'arguments.'
        # print 'Argument List:', str(sys.argv)

        talker()

    except rospy.ROSInterruptException:
        pass
