#include "registration_apps/visualizer_lcm.hpp"

using namespace std;

namespace aicp {

LCMVisualizer::LCMVisualizer(boost::shared_ptr<lcm::LCM>& lcm) : lcm_(lcm)
{}

void LCMVisualizer::publishCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                 int channel,
                                 string name,
                                 int64_t utime = -1)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*cloud, *rgb_cloud);
    drawPointCloudCollections(lcm_, channel, global_, *rgb_cloud, 1, name);
}

void LCMVisualizer::publishCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud,
                                 int channel,
                                 string name,
                                 int64_t utime = -1)
{
    drawPointCloudNormalsCollections(lcm_, channel, global_, *cloud, 0, name);
}

void LCMVisualizer::publishOctree(octomap::ColorOcTree*& octree,
                                  string channel_name)
{
    publishOctreeToLCM(lcm_, octree, channel_name);
}

void LCMVisualizer::publishPose(Isometry3d pose_,
                                int param,
                                string name,
                                int64_t utime = -1)
{
    // TO DO: fix this part
}

}
