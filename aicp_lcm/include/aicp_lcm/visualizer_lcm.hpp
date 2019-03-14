#pragma once

#include "aicp_utils/visualizer.hpp"
#include "aicp_lcm/drawingUtils.hpp"

namespace aicp {

class LCMVisualizer : public Visualizer
{
public:
    LCMVisualizer(boost::shared_ptr<lcm::LCM>& lcm);
    ~LCMVisualizer(){}

    // Publish cloud lcm
    void publishCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                      int param, // channel
                      string name,
                      int64_t utime);
    void publishCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud,
                      int param, // channel
                      string name,
                      int64_t utime);

    // Publish map lcm
    void publishMap(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                    int64_t utime,
                    int channel)
    {}

    // Publish octree lcm
    void publishOctree(octomap::ColorOcTree*& octree,
                       string channel_name);

    // Publish corrected pose lcm
    void publishPoses(Eigen::Isometry3d pose_,
                      int param,
                      string name,
                      int64_t utime);

    // Gets
    const std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>& getPath()
    {}

private:
    boost::shared_ptr<lcm::LCM> lcm_;
};
}
