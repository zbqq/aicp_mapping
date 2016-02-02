#include "pointmatcher/PointMatcher.h"

#include "boost/filesystem.hpp"

#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

#include <cassert>
#include <fstream>
#include <iostream>
#include <algorithm>

#include <math.h>       /* cos, sin */

#include <pcl/common/io.h>

#include <icp-registration/vtkUtils.h>

using namespace std;
using namespace PointMatcherSupport;

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;
typedef PM::Parameters Parameters;

float hausdorffDistance(DP &ref, DP &out);
float hausdorffDistance(DP &ref, DP &out, const char *filename);

float pairedPointsMeanDistance(DP &ref, DP &out, PM::ICP &icp);
float pairedPointsMeanDistance(DP &ref, DP &out, PM::ICP &icp, const char *filename);

string readLineFromFile(string& filename, int line_number);

PM::TransformationParameters parseTransformationDeg(string& transform,
                        const int cloudDimension);
PM::TransformationParameters parseTransformation(string& transform,
                        const int cloudDimension);

void fromDataPointsToPCL(DP &cloud_in, pcl::PointCloud<pcl::PointXYZRGB> &cloud_out);

void writeTransformToFile(Eigen::MatrixXf &transformations, string out_file, int num_clouds);

