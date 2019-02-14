#!/usr/bin/env python
import roslib
import rospy
import numpy

import tf
from geometry_msgs.msg import PoseWithCovarianceStamped
#from geometry_msgs.msg import TransformStamped

def getMatrix(trans,quat):
    matrix = numpy.dot(tf.transformations.translation_matrix(trans), tf.transformations.quaternion_matrix(quat))
    return matrix




def handle_corrected_pose(msg_full):
    msg = msg_full.pose
    tfBroadcaster = tf.TransformBroadcaster()

    try:
        (b2o_trans,b2o_quat) = tfListener.lookupTransform('/odom', '/base', msg_full.header.stamp) # from base --> to odom
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print "return"
        return

    b2o_matrix = getMatrix(b2o_trans,b2o_quat)

    verbose = False

    if (verbose):
        print "============================="
        print msg_full.header.seq
        print b2o_trans
        print b2o_quat
        print tf.transformations.euler_from_quaternion(b2o_quat, 'sxyz')
        print b2o_matrix
        print " "

    # from AICP: from map --> to base
    m2b_trans = [msg.pose.position.x,msg.pose.position.y,msg.pose.position.z]
    m2b_quat = [msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w]

    m2b_matrix = getMatrix(m2b_trans,m2b_quat)

    if (verbose):
        print m2b_trans
        print m2b_quat
        print tf.transformations.euler_from_quaternion(m2b_quat, 'sxyz')
        print m2b_matrix
        print " "


    #######################
    o2b_matrix = tf.transformations.inverse_matrix(b2o_matrix)

    m2o_matrix = numpy.dot(m2b_matrix , o2b_matrix)
    m2o_trans = tf.transformations.translation_from_matrix(m2o_matrix)
    m2o_quat = tf.transformations.quaternion_from_matrix(m2o_matrix)

    if (verbose):
        print m2o_trans
        print m2o_quat
        print tf.transformations.euler_from_quaternion(m2o_quat, 'sxyz')
        print m2o_matrix
        print " "
        print " "

    tfBroadcaster.sendTransform( m2o_trans , m2o_quat,
                                 msg_full.header.stamp,
                                 "odom",       # to frame    <-|
                                 "map_test")   # from frame  --|





if __name__ == '__main__':
    rospy.init_node('aicp_tf_broadcaster')
    tfListener = tf.TransformListener()

    rospy.Subscriber('/aicp/pose_corrected',
                     PoseWithCovarianceStamped,
                     handle_corrected_pose)
    rospy.spin()
