#pragma once

#include "registration_apps/visualizer.hpp"

namespace aicp {

class ROSVisualizer : public Visualizer
{
public:
    ROSVisualizer();
//    ROSVisualizer(boost::shared_ptr<lcm::LCM>& lcm);
//    ~ROSVisualizer();

    // Publish cloud ros
    void publishCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                      int channel,
                      string name);
    void publishCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud,
                      int channel,
                      string name);

    // Publish octree ros
    void publishOctree(octomap::ColorOcTree*& octree,
                       string channel_name);

private:
//    boost::shared_ptr<lcm::LCM> lcm_;
};
}
