#!/usr/bin/env python
# when playing back a bag, filter out odom-to-map from tf
# Note: need to replay the bag while remapping /tf to /tf_old
# rosbag play filename.bag --pause --clock /tf:=/tf_old

import rospy
from tf2_msgs.msg import TFMessage

import tf
import math
import numpy
import time



tf_new_pub = rospy.Publisher("/tf", TFMessage, queue_size=10)


def handle_odom_to_map(msg):

    for t in msg.transforms:
        if (t.child_frame_id == "map"):
            print "y"
            return

    tf_new_pub.publish(msg)






rospy.init_node('aicp_tf_filter')
rospy.Subscriber('/tf_old',
                  TFMessage,
                  handle_odom_to_map)
rospy.spin()
