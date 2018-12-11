#pragma once

#include "registration_apps/visualizer.hpp"

#include "aicp_drawing_utils/drawingUtils.hpp"

namespace aicp {

class LCMVisualizer : public Visualizer
{
public:
    LCMVisualizer(boost::shared_ptr<lcm::LCM>& lcm);
//    ~LCMVisualizer();

    // Publish cloud Director
    void publishCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                      int channel,
                      string name)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::copyPointCloud(*rgb_cloud, *cloud);
        drawPointCloudCollections(lcm_vis_, channel, global_, *rgb_cloud, 1, name);
    }

private:
    boost::shared_ptr<lcm::LCM> lcm_vis_;
};
}
