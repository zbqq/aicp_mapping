# Script to plot pose estimation errors (3D translation & rotation) live using signal-scope
# Run script compute_transl_rot_errors.py to publish 'POSE_ERRORS' message

def makePlots():
    # translation error plot
    addPlot(yLimits=[0, 0.9])
    addSignal('POSE_ERRORS', msg.utime, msg.values[2])
    addSignal('POSE_ERRORS', msg.utime, msg.values[3])
    addSignal('POSE_ERRORS', msg.utime, msg.values[4])
    addSignal('POSE_ERRORS', msg.utime, msg.values[0])

    # rotation error plot
    addPlot(yLimits=[0, 25])
    addSignal('POSE_ERRORS', msg.utime, msg.values[5])
    addSignal('POSE_ERRORS', msg.utime, msg.values[6])
    addSignal('POSE_ERRORS', msg.utime, msg.values[7])

    # rotation error plot
    addPlot(yLimits=[0, 25])
    addSignal('POSE_ERRORS', msg.utime, msg.values[1])

    setFormatOptions(pointSize=4,timeWindow=100.0,curveStyle="lines")

makePlots()
