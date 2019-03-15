#pragma once

#include "ros/node_handle.h"

#include "aicp_utils/visualizer.hpp"

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <eigen_conversions/eigen_msg.h>

#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <Eigen/StdVector>

namespace aicp {

class ROSVisualizer : public Visualizer
{
public:

    ROSVisualizer(ros::NodeHandle& nh, std::string fixed_frame);
    ~ROSVisualizer(){}

    // Publish cloud
    void publishCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                      int param, // channel name
                      std::string name,
                      int64_t utime);
    void publishCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud,
                      int param, // channel name
                      std::string name,
                      int64_t utime);

    // Publish map
    void publishMap(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                    int64_t utime,
                    int channel); // 0 : /aicp/prior_map
                                  // 1 : /aicp/aligned_map

    // Publish octree
    void publishOctree(octomap::ColorOcTree*& octree,
                       std::string channel_name);

    // Publish corrected pose
    void publishPoses(Eigen::Isometry3d pose,
                      int param, std::string name, int64_t utime);

    // Publish tf from fixed_frame to odom
    void publishFixedFrameToOdomTF(Eigen::Isometry3d& fixed_frame_to_base_eigen, ros::Time msg_time);

    // Gets
    const PathPoses& getPath(){
        return path_;
    }

private:
    ros::NodeHandle& nh_;
    ros::Publisher cloud_pub_;
    ros::Publisher prior_map_pub_;
    ros::Publisher aligned_map_pub_;
    ros::Publisher pose_pub_;
    // Duplicates the list in collections renderer. assumed to be 3xN colors
    std::vector<double> colors_;
    // Path (vector of poses)
    PathPoses path_;

    std::string fixed_frame_; // map or map_test
    std::string odom_frame_;
    std::string base_frame_;

    // TF listener and broadcaster
    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;
};
}
