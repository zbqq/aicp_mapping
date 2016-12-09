# Subscribes "COMPUTED_ERROR_TRANSFORM" (from error-from-gt-map)
# Writes 3D translation and 3D rotation errors to file. Columns: # rowIndex, time [s], translErr [m], rotErr [m], updatedCorrectionFlag
# Publishes 3D translation and 3D rotation errors to "POSE_ERRORS"

#!/usr/bin/python
import lcm
import math
import numpy as np
import os

from bot_core.pose_t import pose_t
from bot_core.double_array_t import double_array_t

####################################################################
def quat_to_rot(q) :
  x = q[0];
  y = q[1];
  z = q[2];
  w = q[3];
  R = np.matrix( ((1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w),(2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w),(2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y)) )

  return R

def on_correction(channel, data):
  global updatedCorrection
  updatedCorrection = True

def on_pose(channel, data):
  errorTransform = pose_t.decode(data)

  errorsToPub = double_array_t()
  errorsToPub.utime = errorTransform.utime
  errorsToPub.num_values = 2
  errorsToPub.values = [0] * errorsToPub.num_values

  # Compute 3D translation error
  translErr = math.sqrt(pow(errorTransform.pos[0], 2.0) + pow(errorTransform.pos[1], 2.0) + pow(errorTransform.pos[2], 2.0));

  # Compute 3D rotation error
  errorRotMatrix = quat_to_rot(errorTransform.orientation);

  traceRot = errorRotMatrix.trace();
  rotErr = math.acos ( (traceRot-1)/2 ) * 180.0 / math.pi;

  # Set output variable
  errorsToPub.values[0] = translErr
  errorsToPub.values[1] = rotErr

  # Get time in seconds
  global index
  if index == 0:
    global utimeStart
    utimeStart = errorsToPub.utime

  utimeCurrent = (errorsToPub.utime - utimeStart);
  timeCurrent = utimeCurrent*1e-6;

  # Set updated correction flag
  global updatedCorrection
  updatedFlag = False
  if updatedCorrection:
    updatedFlag = True
    updatedCorrection = False

  # Write to file
  global file
  # rowIndex, time [s], translErr [m], rotErr [m], updatedCorrectionFlag
  file.write('%d \t' % index)
  file.write('%f \t' % timeCurrent)
  file.write('%f \t' % translErr)
  file.write('%f \t' % rotErr)
  file.write('%i \n' % updatedFlag)
  index = index + 1

  # Publish to LCM
  lc.publish("POSE_ERRORS", errorsToPub.encode())

####################################################################
lc = lcm.LCM()
print "Compute Pose Errors: started..."
index = 0
utimeStart = 0
updatedCorrection = False

fileName = 'errorsFromTransformFile.txt'
currentDir = os.path.dirname(os.path.abspath(__file__))
destDir = os.path.join(currentDir, 'results')
try:
    os.makedirs(destDir)
except OSError:
    pass # already exists
path = os.path.join(destDir, fileName)
file = open(path, 'w')
print "Writing to: %s" % path

sub1 = lc.subscribe("COMPUTED_ERROR_TRANSFORM", on_pose)
sub2 = lc.subscribe("POSE_BODY_SCANMATCHER", on_correction)
while True:
  lc.handle()

lc.unsubscribe(sub1)
lc.unsubscribe(sub2)
file.close()
