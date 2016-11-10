# Subscribes 'POSE_BODY' (from SE) and 'POSE_GROUND_TRUTH' (from Vicon)
# Publishes 3D translation and 3D rotation errors

#!/usr/bin/python
import lcm
import math
import numpy as np

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

def on_vicon(channel, data):
  global viconPose
  viconPose = pose_t.decode(data)

def on_pose(channel, data):
  estimatedPose = pose_t.decode(data)

  if viconPose != 0:
    errors = double_array_t()
    errors.utime = estimatedPose.utime
    errors.num_values = 2
    errors.values = [0] * errors.num_values

    transErr = math.sqrt( pow(viconPose.pos[0] - estimatedPose.pos[0], 2.0) + pow(viconPose.pos[1] - estimatedPose.pos[1], 2.0) + pow(viconPose.pos[2] - estimatedPose.pos[2], 2.0));

    viconRot = quat_to_rot(viconPose.orientation);
    estimatedRot = quat_to_rot(estimatedPose.orientation);

    try:
        viconRotInverse = np.linalg.inv(viconRot)
    except np.linalg.LinAlgError:
        # Not invertible. Skip this one.
        pass

    deltaRot = np.dot(estimatedRot, viconRotInverse)
    traceRot = deltaRot.trace();
    rotErr = math.acos ( (traceRot-1)/2 ) * 180.0 / math.pi;

    errors.values[0] = transErr
    errors.values[1] = rotErr

    lc.publish("POSE_ERRORS", errors.encode())

####################################################################
lc = lcm.LCM()
print "Compute Pose Errors: started..."
viconPose = 0
sub1 = lc.subscribe("POSE_BODY", on_pose)
sub2 = lc.subscribe("POSE_GROUND_TRUTH", on_vicon)
while True:
  lc.handle()

lc.unsubscribe(sub1)
lc.unsubscribe(sub2)
