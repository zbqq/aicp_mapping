#ifndef AICP_POINTMATCHER_REGISTRATION_HPP_
#define AICP_POINTMATCHER_REGISTRATION_HPP_

//libpointmatcher
//#include "pointmatcher/PointMatcher.h"
//Eigen
#include <Eigen/Eigenvalues>

//Project lib
#include "aicpRegistration/common.hpp"
#include "aicpRegistration/abstract_registrator.hpp"

#include "commonUtils/cloudIO.h"
#include "commonUtils/fileIO.h"
#include "pointmatcherUtils/icpMonitor.h"

using namespace std;

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;

namespace aicp{

class PointmatcherRegistration : public AbstractRegistrator {
  public:
    PointmatcherRegistration();
    explicit PointmatcherRegistration(const RegistrationParams& params);
    ~PointmatcherRegistration();

    virtual void registerClouds(pcl::PointCloud<pcl::PointXYZ>& cloud_ref, pcl::PointCloud<pcl::PointXYZ>& cloud_read, Eigen::Matrix4f &final_transform);
    virtual void registerClouds(pcl::PointCloud<pcl::PointXYZRGB>& cloud_ref, pcl::PointCloud<pcl::PointXYZRGB>& cloud_read, Eigen::Matrix4f &final_transform);
    virtual void registerClouds(pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud_ref, pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud_read, Eigen::Matrix4f &final_transform);

    void applyConfig();
    void applyInitialization();

    //PM::ICP getIcp(){ return icp_; }

    void getInitializedReading(pcl::PointCloud<pcl::PointXYZ>& initialized_reading)
    {
      fromDataPointsToPCL(initialized_reading_, initialized_reading);
    }

    void getOutputReading(pcl::PointCloud<pcl::PointXYZ>& out_read_cloud){
      fromDataPointsToPCL(out_read_cloud_, out_read_cloud);
    }

    void updateConfigParams(string config_name){
      params_.pointmatcher.configFileName.clear();
      params_.pointmatcher.configFileName.append(config_name);}

  private:
    RegistrationParams params_;

    void registerClouds(Eigen::Matrix4f &final_transform);

    PM::ICP icp_;
  
    DP reference_cloud_;
    DP reading_cloud_;
    DP initialized_reading_;
    DP out_read_cloud_;
    PM::TransformationParameters init_transform_;
};

}

#endif
