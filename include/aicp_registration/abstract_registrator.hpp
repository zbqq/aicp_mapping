#ifndef AICP_REGISTRATOR_ABSTRACT_HPP_
#define AICP_REGISTRATOR_ABSTRACT_HPP_

#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>

namespace aicp {
  class AbstractRegistrator {
  public:
    virtual void registerClouds(pcl::PointCloud<pcl::PointXYZ>& cloud_ref, pcl::PointCloud<pcl::PointXYZ>& cloud_read, Eigen::Matrix4f &final_transform) = 0;
    virtual void registerClouds(pcl::PointCloud<pcl::PointXYZRGB>& cloud_ref, pcl::PointCloud<pcl::PointXYZRGB>& cloud_read, Eigen::Matrix4f &final_transform) = 0;
    virtual void registerClouds(pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud_ref, pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud_read, Eigen::Matrix4f &final_transform) = 0;

    virtual void getInitializedReading(pcl::PointCloud<pcl::PointXYZ>& initialized_reading) = 0;
    virtual void getOutputReading(pcl::PointCloud<pcl::PointXYZ>& out_read_cloud) = 0;

    virtual void updateConfigParams(std::string config_name) = 0;

  };
}

#endif
