# Subscribes "POSE_ERRORS"
# Writes 3D translation and 3D rotation errors to file. Columns: # rowIndex, time [s], translErr [m], rotErr [deg]

#!/usr/bin/python
import lcm
import os

from bot_core.double_array_t import double_array_t

####################################################################
def on_errors(channel, data):
  errors = double_array_t.decode(data)

  # Get time in seconds
  global index
  if index == 0:
    global utimeStart
    utimeStart = errors.utime

  utimeCurrent = (errors.utime - utimeStart);
  timeCurrent = utimeCurrent*1e-6;

  # Write to file
  global file
  # rowIndex, time [s], translErr [m], rotErr [deg]
  file.write('%d \t' % index)
  file.write('%f \t' % timeCurrent)
  file.write('%f \t' % errors.values[0])
  file.write('%f \n' % errors.values[1])
  index = index + 1

####################################################################
lc = lcm.LCM()
print "Errors to file: started..."
index = 0
utimeStart = 0

fileName = 'errorsFile.txt'
currentDir = os.path.dirname(os.path.abspath(__file__))
destDir = os.path.join(currentDir, 'results')
try:
    os.makedirs(destDir )
except OSError:
    pass # already exists
path = os.path.join(destDir, fileName)
file = open(path, 'w')
print "Writing to: %s" % path

sub1 = lc.subscribe("POSE_ERRORS", on_errors)
while True:
  lc.handle()

lc.unsubscribe(sub1)
file.close()
