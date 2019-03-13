#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import sys
import math
import numpy as np

pub = rospy.Publisher('/ihmc_ros/localization/pelvis_odom_pose_correction', Odometry, queue_size=10)

cmdargs = sys.argv

change_yaw = 0
change_x = 0
if len(sys.argv) ==3:
    change_yaw = float(cmdargs[1])
    change_x = float(cmdargs[2])
else:
    print "script requires two arguments:"
    print " change in yaw - in degrees"
    print " change in x position - in metres"
    exit(-1)

def quat_to_euler(q) :
  roll_a = 2.0 * (q[0]*q[1] + q[2]*q[3]);
  roll_b = 1.0 - 2.0 * (q[1]*q[1] + q[2]*q[2]);
  roll = math.atan2 (roll_a, roll_b);

  pitch_sin = 2.0 * (q[0]*q[2] - q[3]*q[1]);
  pitch = math.asin (pitch_sin);

  yaw_a = 2.0 * (q[0]*q[3] + q[1]*q[2]);
  yaw_b = 1.0 - 2.0 * (q[2]*q[2] + q[3]*q[3]);  
  yaw = math.atan2 (yaw_a, yaw_b);
  return [roll,pitch,yaw]

def euler_to_quat(rpy):
  roll =  rpy[0]
  pitch = rpy[1]
  yaw =   rpy[2]

  sy = math.sin(yaw*0.5);
  cy = math.cos(yaw*0.5);
  sp = math.sin(pitch*0.5);
  cp = math.cos(pitch*0.5);
  sr = math.sin(roll*0.5);
  cr = math.cos(roll*0.5);
  w = cr*cp*cy + sr*sp*sy;
  x = sr*cp*cy - cr*sp*sy;
  y = cr*sp*cy + sr*cp*sy;
  z = cr*cp*sy - sr*sp*cy;
  return np.array([w,x,y,z])

def callback(m):

    orientation = [m.pose.pose.orientation.w, m.pose.pose.orientation.x, m.pose.pose.orientation.y, m.pose.pose.orientation.z]
    rpy = quat_to_euler(orientation)
    change_yaw_rad = change_yaw*math.pi/180.0
    print "current heading " + str(rpy[2]) +  "(" + str(rpy[2]*180.0/math.pi) + " deg)"
    rpy[2] = rpy[2] + change_yaw_rad
    print "set heading to " + str(rpy[2]) +  "(" + str(rpy[2]*180.0/math.pi) + " deg) - a change of " + str(change_yaw_rad) + " rad (" + str(change_yaw) + " deg)"
    print "change x position by " + str(change_x)
    orientation = euler_to_quat(rpy)  
    m.pose.pose.orientation.w = orientation[0]
    m.pose.pose.orientation.x = orientation[1]
    m.pose.pose.orientation.y = orientation[2]
    m.pose.pose.orientation.z = orientation[3]

    m.pose.pose.position.x = m.pose.pose.position.x + change_x
    pub.publish(m)

    print "Sent pose correction, exiting"
    rospy.signal_shutdown("Exiting")

def listener():
    rospy.init_node('send_fake_correction_ros', anonymous=True)
    rospy.Subscriber("/ihmc_ros/valkyrie/output/robot_pose", Odometry, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()