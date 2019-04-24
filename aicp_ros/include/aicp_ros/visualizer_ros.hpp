#pragma once

#include "ros/node_handle.h"

#include "aicp_utils/visualizer.hpp"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
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

    // Publish corrected poses
    void publishPoses(Eigen::Isometry3d pose,
                      int param, std::string name, int64_t utime);
    void publishPoses(PathPoses poses,
                      int param, std::string name, int64_t utime);

    void publishOdomPoses(Eigen::Isometry3d pose,
                      int param, std::string name, int64_t utime);
    void publishOdomPoses(PathPoses poses,
                      int param, std::string name, int64_t utime);

    void publishPriorPoses(Eigen::Isometry3d pose,
                      int param, std::string name, int64_t utime);
    void publishPriorPoses(PathPoses poses,
                      int param, std::string name, int64_t utime);

    void publishOdomToMapPose(Eigen::Isometry3d pose, int64_t utime);


    // Publish tf from fixed_frame to odom
    void publishFixedFrameToOdomTF(const Eigen::Isometry3d& fixed_frame_to_base_eigen,
                                   ros::Time msg_time);
    void publishFixedFrameToOdomPose(const Eigen::Isometry3d& fixed_frame_to_base_eigen,
                                     ros::Time msg_time);

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
    ros::Publisher odom_pose_pub_;
    ros::Publisher prior_pose_pub_;

    ros::Publisher fixed_to_odom_pub_;
    ros::Publisher odom_to_map_pub_;
    
    // Duplicates the list in collections renderer. assumed to be 3xN colors
    std::vector<double> colors_;
    // Path (vector of poses)
    PathPoses path_;
    PathPoses odom_path_;
    PathPoses prior_path_;

    std::string fixed_frame_; // map or map_test
    std::string odom_frame_;
    std::string base_frame_;
    std::string fixed_to_odom_prefix_ = "/localization_manager/";

    // TF listener and broadcaster
    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;

    geometry_msgs::PoseWithCovarianceStamped fixed_to_odom_msg_;
    tf::Pose temp_tf_pose_;

    void computeFixedFrameToOdom(const Eigen::Isometry3d &fixed_frame_to_base_eigen,
                                 Eigen::Isometry3d& fixed_frame_to_odom_eigen);
};
}
