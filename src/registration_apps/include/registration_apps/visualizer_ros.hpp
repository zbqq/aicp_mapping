#pragma once

#include "ros/node_handle.h"

#include "registration_apps/visualizer.hpp"

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>



namespace aicp {

class ROSVisualizer : public Visualizer
{
public:

    ROSVisualizer(ros::NodeHandle& nh);
//    ~ROSVisualizer();

    // Publish cloud ros
    void publishCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                      int param, // channel name
                      string name,
                      int64_t utime);
    void publishCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud,
                      int param, // channel name
                      string name,
                      int64_t utime);

    // Publish octree ros
    void publishOctree(octomap::ColorOcTree*& octree,
                       string channel_name);

    // Publish corrected pose
    void publishPose(Eigen::Isometry3d pose,
                     int param, string name, int64_t utime);

private:
    ros::NodeHandle& nh_;
    ros::Publisher cloud_pub_;
    ros::Publisher pose_pub_;
    // Duplicates the list in collections renderer. assumed to be 3xN colors
    std::vector<double> colors_;
    // Path (vector of poses)
    std::vector<Eigen::Isometry3d> path_;


};
}
