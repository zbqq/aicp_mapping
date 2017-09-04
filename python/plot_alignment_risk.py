def makePlots():
    addPlot(yLimits=[0, 100])
    # Octree-based overlap parameter
    addSignal('OVERLAP', msg.utime, msg.values[0], None, [0,1,0], 'Octree-based Overlap')

    # Alignability parameter
    addPlot(yLimits=[0, 60])
    addSignal('ALIGNABILITY', msg.utime, msg.values[0], None, [0,1,0], 'Alignability')

    # Alignability Risk factor
    addPlot(yLimits=[0.47, 0.51])
    addSignal('ALIGNMENT_RISK', msg.utime, msg.values[0], None, [1,0,0], 'Alignment Risk')

    # Degeneracy factor
    addPlot(yLimits=[-0.1, 2])
    addSignal('DEGENERACY', msg.utime, msg.values[0], None, [1,0,0], 'Degeneracy')

    setFormatOptions(pointSize=4,timeWindow=250.0,curveStyle="lines")

makePlots()
