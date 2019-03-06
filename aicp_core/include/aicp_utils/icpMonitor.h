#ifndef POINTMATCHER_UTILS_HPP_
#define POINTMATCHER_UTILS_HPP_

#include <fstream>
#include <iostream>

#include <pwd.h>

//libpointmatcher
#include "pointmatcher/PointMatcher.h"

//Project lib
#include <aicp_utils/vtkUtils.h>


using namespace std;
//using namespace PointMatcherSupport;

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;

float hausdorffDistance(DP &ref, DP &out);
float hausdorffDistance(DP &ref, DP &out, const char *filename);

PM::Matrix distancesKNN(DP &A, DP &B);
PM::Matrix distancesKNN(DP &A, DP &B, const char *filename);

float pairedPointsMeanDistance(DP &ref, DP &out, PM::ICP &icp);
float pairedPointsMeanDistance(DP &ref, DP &out, PM::ICP &icp, const char *filename);

void getResidualError(PM::ICP &icp, float overlap, float &meanDist, float &medDist, float &quantDist);

Eigen::Isometry3d getTransfParamAsIsometry3d(PM::TransformationParameters T);

#endif
