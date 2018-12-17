#include "registration_apps/visualizer_ros.hpp"

#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>

using namespace std;

namespace aicp {

ROSVisualizer::ROSVisualizer(ros::NodeHandle& nh) : nh_(nh)
{}

void ROSVisualizer::publishCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                 int param, // buffer size
                                 string name,
                                 int64_t utime = -1)
{
    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(name, param);

    int secs = utime * 1E-6;
    int nsecs = (utime - (secs * 1E6)) * 1E3;

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header.stamp = ros::Time(secs, nsecs);
    output.header.frame_id = "odom";
    cloud_pub_.publish(output);
}

void ROSVisualizer::publishCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud,
                                 int param, // buffer size
                                 string name,
                                 int64_t utime = -1)
{
    // to be implemented
}

void ROSVisualizer::publishOctree(octomap::ColorOcTree*& octree,
                                  string channel_name)
{
    // publishOctree to be implemented.
}
}
