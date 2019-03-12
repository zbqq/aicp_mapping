#ifndef AICP_REGISTRATOR_COMMON_HPP_
#define AICP_REGISTRATOR_COMMON_HPP_

#include <iostream>
#include <string>

struct RegistrationParams {
  typedef std::string string;
  string type = "";
  float sensorRange = -1;
  float sensorAngularView = -1;
  string loadPosesFrom = "";
  string initialTransform = ""; //initial transformation for the reading cloud in the form [x,y,theta]
                                //Note: either this or PointmatcherRegistrationParams.initialTransform
                                //should be set (not both)

    struct PointmatcherRegistrationParams
    {
      string configFileName = "";
      string initialTransform = ""; //initial transformation for the reading cloud in the form [x,y,theta]
      bool printOutputStatistics = false; //e.g. Hausdorff distance, residual mean distance
    } pointmatcher;
};

#endif
