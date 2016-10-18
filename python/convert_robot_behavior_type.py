#!/usr/bin/python
import lcm

from drc.behavior_t import behavior_t
from drc.controller_status_t import controller_status_t

from bot_core.double_array_t import double_array_t

####################################################################
def on_atlas_behavior(channel, data):
 
  beh = controller_status_t.decode(data)

  arr = double_array_t()
  arr.utime = beh.utime
  arr.num_values = 2
  arr.values = [0] * arr.num_values

  if (beh.state == 2): # Walking
    arr.values[1] = 1
  else:
    arr.values[1] = 0  # Standing, manipulating, ...

  print "arr.values[1]: "
  print arr.values[1]

  lc.publish("CURRENT_ROBOT_BEHAVIOR", arr.encode()) 

def on_val_behavior(channel, data):
 
  beh = behavior_t.decode(data)

  arr = double_array_t()
  arr.utime = beh.utime
  arr.num_values = 2
  arr.values = [0] * arr.num_values

  if (beh.behavior == 4): # Walking
    arr.values[1] = 1
  else:
    arr.values[1] = 0  # Standing, manipulating, ...

  lc.publish("CURRENT_ROBOT_BEHAVIOR", arr.encode())

####################################################################
lc = lcm.LCM()
print "Encoding robot behavior state in double array."
sub1 = lc.subscribe("CONTROLLER_STATUS", on_atlas_behavior)
sub2 = lc.subscribe("ROBOT_BEHAVIOR", on_val_behavior)
while True:
  lc.handle()

lc.unsubscribe(sub1)
lc.unsubscribe(sub2)
