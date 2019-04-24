#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <octomap/ColorOcTree.h>

#include <Eigen/Dense>

namespace aicp {

class Visualizer
{
public:
    typedef std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> PathPoses;
public:
    Visualizer(){}
    virtual ~Visualizer(){}

    virtual void publishCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                              int param,
                              std::string name,
                              int64_t utime = -1) = 0;

    virtual void publishCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud,
                              int param,
                              std::string name,
                              int64_t utime = -1) = 0;

    virtual void publishMap(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                            int64_t utime,
                            int channel) = 0;

    virtual void publishOctree(octomap::ColorOcTree*& octree,
                               std::string channel_name) = 0;

    virtual void publishPoses(Eigen::Isometry3d pose_,
                              int param,
                              std::string name,
                              int64_t utime = -1) = 0;

    virtual void publishOdomPoses(Eigen::Isometry3d pose_,
                              int param,
                              std::string name,
                              int64_t utime = -1) = 0;

    virtual void publishPriorPoses(Eigen::Isometry3d pose_,
                              int param,
                              std::string name,
                              int64_t utime = -1) = 0;

    virtual void publishOdomToMapPose(Eigen::Isometry3d pose_,
                              int64_t utime = -1) = 0;


    // Gets
    virtual const PathPoses& getPath() = 0;

protected:
    // Set global reference frame to zero origin
    Eigen::Isometry3d global_ = Eigen::Isometry3d::Identity();
};
}
