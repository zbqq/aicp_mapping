#ifndef AICP_REGISTRATOR_COMMON_HPP_
#define AICP_REGISTRATOR_COMMON_HPP_

#include <iostream>
#include <string>

using namespace std;

struct RegistrationParams {
  string type;
  float sensorRange;
  float sensorAngularView;
  string loadPosesFromFile;
  string initialTransform; //initial transformation for the reading cloud in the form [x,y,theta]
                           //Note: either this or PointmatcherRegistrationParams.initialTransform
                           //should be set (not both)
  bool saveCorrectedPose;
  bool saveInitializedReadingCloud;
  bool saveRegisteredReadingCloud;
  bool enableLcmVisualization;

    struct PointmatcherRegistrationParams
    {
      string configFileName;
      string initialTransform; //initial transformation for the reading cloud in the form [x,y,theta]
      bool printOutputStatistics; //e.g. Hausdorff distance, residual mean distance
    } pointmatcher;
};

#endif
