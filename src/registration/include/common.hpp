#ifndef AICP_REGISTRATOR_COMMON_HPP_
#define AICP_REGISTRATOR_COMMON_HPP_

#include <iostream>
#include <string>
//#include <vector>
//#include <Eigen/Dense>
//#include <pcl/point_types.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/common/common_headers.h>

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
    } pointmatcher;

};

#endif
