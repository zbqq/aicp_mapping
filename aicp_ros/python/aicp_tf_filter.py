#!/usr/bin/env python
# link aicp's correction into tf (instead of localization manager)

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
