#ifndef MIXED_TEST_ROUTINE_HPP_
#define MIXED_TEST_ROUTINE_HPP_

#include "ioUtils/fileIO.h"
#include "pointmatcherUtils/icpMonitor.h"
//#include "cloud_accumulate/cloud_accumulate.hpp"

//#include "boost/filesystem.hpp"

//#include <unistd.h>
//#include <sys/types.h>
//#include <pwd.h>

//#include <cassert>
//#include <fstream>
//#include <iostream>
//#include <algorithm>

//#include <iomanip>
//#include <random>

//#include <pcl/common/io.h>

//using namespace std;
//using namespace PointMatcherSupport;

//typedef PointMatcher<float> PM;
//typedef PM::DataPoints DP;
//typedef PM::Parameters Parameters;


class RoutineConfig
{
  public:
    RoutineConfig();

    const char *homedir;

    string initFilename; //name of file containing initial transf
    string initTrans; //initial transformation for the reading cloud in the form [x,y,theta]

    int num_clouds; //number of clouds to combine

  private:
};

class RegistrationRoutine
{
  public:
    RegistrationRoutine();
    void init();
    
    ~RegistrationRoutine();

    string configFile2D; //name of the config file defining the ICP chain parameters (for 2D matching).
    string configFile3D; //name of the config file defining the ICP chain parameters (for 3D matching).

    int validateArgs(const int argc, const char *argv[]);
    void usage(const char *argv[]);

    int cols; //number of columns in output matrix (number performed registrations)

    string cloud_name_A; //reference cloud file name
    string cloud_name_B; //input cloud file name

    int cloudDim; //dimension of the reference cloud

    void doRoutine(Eigen::MatrixXf &transf_matrix);
  
    RoutineConfig cfg_;
  private:
    void getICPTransform(DP &cloud_in, DP &cloud_ref, PM::TransformationParameters &T, DP &cloud_out, PM::ICP &icp);
};


#endif
