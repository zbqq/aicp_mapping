#ifndef AICP_REGISTRATOR_ABSTRACT_HPP_
#define AICP_REGISTRATOR_ABSTRACT_HPP_

//#include <Eigen/Dense>
#include <pcl/point_types.h>
//#include <pcl/io/pcd_io.h>
#include <pcl/common/common_headers.h>

namespace aicp {
  class AbstractRegistrator {
  public:
    virtual void registerClouds(const pcl::PointCloud<pcl::PointXYZ>& cloud_ref, const pcl::PointCloud<pcl::PointXYZ>& cloud_read, Eigen::Matrix4f &final_transform) = 0;
    virtual void registerClouds(const pcl::PointCloud<pcl::PointXYZRGB>& cloud_ref, const pcl::PointCloud<pcl::PointXYZRGB>& cloud_read, Eigen::Matrix4f &final_transform) = 0;
    virtual void registerClouds(const pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud_ref, const pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud_read, Eigen::Matrix4f &final_transform) = 0;
  };
}

#endif
