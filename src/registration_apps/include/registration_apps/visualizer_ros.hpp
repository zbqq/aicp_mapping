#pragma once

#include "ros/node_handle.h"

#include "registration_apps/visualizer.hpp"


namespace aicp {

class ROSVisualizer : public Visualizer
{
public:

    ROSVisualizer(ros::NodeHandle& nh);
//    ~ROSVisualizer();

    // Publish cloud ros
    void publishCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                      int param, // buffer size
                      string name,
                      int64_t utime);
    void publishCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud,
                      int param, // buffer size
                      string name,
                      int64_t utime);

    // Publish octree ros
    void publishOctree(octomap::ColorOcTree*& octree,
                       string channel_name);

private:
    ros::NodeHandle& nh_;
    ros::Publisher cloud_pub_;
    // Duplicates the list in collections renderer. assumed to be 3xN colors
       std::vector < double > colors_;

};
}
