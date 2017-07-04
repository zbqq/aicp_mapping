#ifndef AICP_REGISTRATOR_COMMON_HPP_
#define AICP_REGISTRATOR_COMMON_HPP_

#include <iostream>
#include <string>

using namespace std;

struct RegistrationParams {
  std::string type;
  bool saveTransform;
  bool saveInitializedReadingCloud;
  bool saveRegisteredReadingCloud;

    struct PointmatcherRegistrationParams
    {
      string configFileName;
      string initialTransform; //initial transformation for the reading cloud in the form [x,y,theta]
      bool printOutputStatistics; //e.g. Hausdorff distance, residual mean distance
    } pointmatcher;

};

#endif
