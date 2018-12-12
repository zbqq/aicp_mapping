#pragma once

#include "registration_apps/visualizer.hpp"

#include "aicp_drawing_utils/drawingUtils.hpp"

namespace aicp {

class LCMVisualizer : public Visualizer
{
public:
    LCMVisualizer(boost::shared_ptr<lcm::LCM>& lcm);
//    ~LCMVisualizer();

    // Publish cloud lcm
    void publishCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                      int channel,
                      string name);
    void publishCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud,
                      int channel,
                      string name);

    // Publish octree lcm
    void publishOctree(octomap::ColorOcTree*& octree,
                       string channel_name);

private:
    boost::shared_ptr<lcm::LCM> lcm_;
};
}
