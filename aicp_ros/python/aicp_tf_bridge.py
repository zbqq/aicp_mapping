#!/usr/bin/env python
# link aicp's correction into tf (instead of localization manager)

import rospy
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import *
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker

import tf
import math
import numpy
import time




pub = rospy.Publisher("/tf", TFMessage, queue_size=10)
vis_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=10)
rospy.loginfo("aicp_tf_bridge")


def handle_odom_to_map(msg):
    print "x"

    #
    tfm = TFMessage()
    this_t = TransformStamped()
    this_t.header = msg.header
    this_t.header.frame_id = "odom"
    this_t.child_frame_id = "map"
    this_t.transform.translation = msg.pose.pose.position
    this_t.transform.rotation = msg.pose.pose.orientation
    tfm.transforms.append(this_t)

    pub.publish(tfm)






rospy.init_node('aicp_tf_bridge')
rospy.Subscriber('/icp_tools/map_pose',
                  PoseWithCovarianceStamped,
                  handle_odom_to_map)
rospy.spin()
