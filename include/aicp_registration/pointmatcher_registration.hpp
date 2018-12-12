#ifndef AICP_POINTMATCHER_REGISTRATION_HPP_
#define AICP_POINTMATCHER_REGISTRATION_HPP_

//libpointmatcher
//#include "pointmatcher/PointMatcher.h"
//Eigen
#include <Eigen/Eigenvalues>

//Project lib
#include "aicp_registration/common.hpp"
#include "aicp_registration/abstract_registrator.hpp"

#include "aicp_common_utils/cloudIO.h"
#include "aicp_common_utils/fileIO.h"
#include "aicp_pointmatcher_utils/icpMonitor.h"
#include "aicp_filtering_utils/filteringUtils.hpp"

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
    PM::TransformationParameters applyInitialization();

//    PM::ICP getIcp(){ return icp_; }

    void getInitializedReading(pcl::PointCloud<pcl::PointXYZ>& initialized_reading)
    {
      if (initialized_reading_.getNbPoints() > 0)
        fromDataPointsToPCL(initialized_reading_, initialized_reading);
      else
      {
        std::cout << "[Pointmatcher] Reading cloud not initialized here." << std::endl;
        fromDataPointsToPCL(reading_cloud_, initialized_reading);
      }
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
};

}

#endif
