#!/usr/bin/python
import lcm
import time
import math
import numpy  as np

import botpy
from bot_core.pose_t import pose_t

####################################################################
def timestamp_now (): return int (time.time () * 1000000)

def on_pose(channel, data):

  m = pose_t.decode(data)

  rpy = botpy.quat_to_euler(m.orientation)
  print rpy[2]*180.0/math.pi

  rpy[2] = rpy[2]#- 20.0*math.pi/180.0
  print rpy[2]*180.0/math.pi
  m.orientation = botpy.euler_to_quat(rpy)  

  pos = list(m.pos)
  pos[0] = pos[0] - 0.05
  print pos[0]
  m.pos = pos

  lc.publish("POSE_BODY_CORRECTED", m.encode()) 
  quit()

####################################################################
lc = lcm.LCM()
print "started"
sub1 = lc.subscribe("POSE_BODY", on_pose)
while True:
  lc.handle()
lc.unsubscribe(sub1)