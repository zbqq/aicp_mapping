#pragma once

#include "ros/node_handle.h"

#include "registration_apps/visualizer.hpp"

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

namespace aicp {

class ROSVisualizer : public Visualizer
{
public:

    ROSVisualizer(ros::NodeHandle& nh, string fixed_frame);
//    ~ROSVisualizer();

    // Publish cloud
    void publishCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                      int param, // channel name
                      string name,
                      int64_t utime);
    void publishCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud,
                      int param, // channel name
                      string name,
                      int64_t utime);

    // Publish map
    void publishMap(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                    int64_t utime);

    // Publish octree
    void publishOctree(octomap::ColorOcTree*& octree,
                       string channel_name);

    // Publish corrected pose
    void publishPose(Eigen::Isometry3d pose,
                     int param, string name, int64_t utime);

    // Publish tf from fixed_frame to odom
    void publishFixedFrameToOdomTF(Eigen::Isometry3d& fixed_frame_to_base_eigen, ros::Time msg_time);

private:
    ros::NodeHandle& nh_;
    ros::Publisher cloud_pub_;
    ros::Publisher map_pub_;
    ros::Publisher pose_pub_;
    // Duplicates the list in collections renderer. assumed to be 3xN colors
    std::vector<double> colors_;
    // Path (vector of poses)
    std::vector<Eigen::Isometry3d> path_;

    std::string fixed_frame_; // map or map_test
    std::string odom_frame_;
    std::string base_frame_;

    // TF listener and broadcaster
    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;
};
}
