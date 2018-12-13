#include "registration_apps/visualizer_ros.hpp"

using namespace std;

namespace aicp {

//LCMVisualizer::LCMVisualizer(boost::shared_ptr<lcm::LCM>& lcm) : lcm_(lcm)
//{}

ROSVisualizer::ROSVisualizer()
{}

void ROSVisualizer::publishCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                 int channel,
                                 string name)
{
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//    pcl::copyPointCloud(*cloud, *rgb_cloud);
//    drawPointCloudCollections(lcm_, channel, global_, *rgb_cloud, 1, name);
}

void ROSVisualizer::publishCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud,
                                 int channel,
                                 string name)
{
//    drawPointCloudNormalsCollections(lcm_, channel, global_, *cloud, 0, name);
}

void ROSVisualizer::publishOctree(octomap::ColorOcTree*& octree,
                                  string channel_name)
{
//    publishOctreeToLCM(lcm_, octree, channel_name);
}
}
