#pragma once

#include "aicp_drawing_utils/drawingUtils.hpp"

namespace aicp {

class Visualizer
{
public:
    Visualizer(){

    }
//    ~Visualizer();

protected:
    // Set global reference frame to zero origin
    Eigen::Isometry3d global_ = Eigen::Isometry3d::Identity();

    virtual void publishCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                              int channel,
                              string name) = 0;
};
}
