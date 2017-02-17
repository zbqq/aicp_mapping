#!/usr/bin/python
import lcm
import math
import numpy as np
import os
import time

from bot_core.pose_t import pose_t
from bot_core.double_array_t import double_array_t


####################################################################
# globals
global correctionTime, initialPoseBodyMsg, counterPosesAfterInitial, currentError, highestError

correctionTime = None
initialPoseBodyMsg = None
counterPosesAfterInitial = 0
currentError = 10000
highestError = 0

global index, utimeStart

index = 0
utimeStart = None

####################################################################
def quat_to_rot(q) :
  x = q[0];
  y = q[1];
  z = q[2];
  w = q[3];
  R = np.matrix( ((1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w),(2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w),(2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y)) )

  return R

def publish_error(prevPose, currPose):
  global correctionTime, initialPoseBodyMsg, counterPosesAfterInitial, highestError, index, utimeStart

  #errorsToPub.values.append((currPose.pos[0] - prevPose.pos[0])); # x
  #errorsToPub.values.append((currPose.pos[1] - prevPose.pos[1])); # y
  #errorsToPub.values.append((currPose.pos[2] - prevPose.pos[2])); # z
  xDifference = (currPose.pos[0] - prevPose.pos[0]) # x
  yDifference = (currPose.pos[1] - prevPose.pos[1]) # y
  zDifference = (currPose.pos[2] - prevPose.pos[2]) # z

  translErr = math.sqrt(pow(xDifference, 2.0) + pow(yDifference, 2.0) + pow(zDifference, 2.0));

  if translErr > highestError:
    highestError = translErr

  if counterPosesAfterInitial == 100:
    print 'publishing.....'
    initialPoseBodyMsg = None
    counterPosesAfterInitial = 0

    # publish
    errorsToPub = double_array_t()
    errorsToPub.utime = currPose.utime

    errorsToPub.values.append(xDifference)
    errorsToPub.values.append(yDifference)
    errorsToPub.values.append(zDifference)
    errorsToPub.values.append(highestError)

    errorsToPub.num_values = len(errorsToPub.values)

    lc.publish("POSE_ERRORS", errorsToPub.encode())

    # Get time in seconds
    utimeCurrent = (errorsToPub.utime - utimeStart);
    timeCurrent = utimeCurrent*1e-6;

    # write to file
    global file
    # rowIndex, time [s], translErr [m]
    file.write('%d \t' % index)
    file.write('%f \t' % timeCurrent)
    file.write('%f \n' % highestError)
    index = index + 1

    print 'highestError: '
    print highestError
    highestError = 0
    print '==================================================='




def on_pb(channel, data):
  # record a queue of messages and compute the largest translational error between them, display that.
  global correctionTime, initialPoseBodyMsg, counterPosesAfterInitial
  msg = pose_t.decode(data)

  # Get time start
  global utimeStart
  if utimeStart == None:
      utimeStart = msg.utime

  # if there is a correction, record the current pose body we know that the next
  # measurement will be a correction, therefore we need to store this pose, in
  # order to estimate the trajectory
  if msg.utime > correctionTime and correctionTime != None:
    if initialPoseBodyMsg == None:
      initialPoseBodyMsg = msg
      return
    # once we have received a correction - don't record the first pose body
    correctionTime = None

  # now check if we have received a pose body initially, only then compute the
  # difference and publish it
  if initialPoseBodyMsg != None:
    # compute and publish the diff
    counterPosesAfterInitial = counterPosesAfterInitial + 1
    publish_error(initialPoseBodyMsg, msg)





def on_correction(channel, data):
  global correctionTime
	# record the timestamp at which we have received the message
  msg = pose_t.decode(data)
  correctionTime = msg.utime


####################################################################
print "Corrections magnitude to file: started..."

fileName = 'corrMagnitudeFileAfterEKF.txt'
currentDir = os.path.dirname(os.path.abspath(__file__))
destDir = os.path.join(currentDir, 'results')
try:
    os.makedirs(destDir )
except OSError:
    pass # already exists
path = os.path.join(destDir, fileName)
file = open(path, 'w')
print "Writing to: %s" % path


lc = lcm.LCM()

sub1 = lc.subscribe("POSE_BODY", on_pb)
sub2 = lc.subscribe("POSE_BODY_SCANMATCHER", on_correction)
while True:
  lc.handle()
