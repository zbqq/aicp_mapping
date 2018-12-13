#pragma once

#include <ros/node_handle.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>

#include "registration_apps/app.hpp"
#include "registration_apps/ros_laser_scan_accumulator.hpp"
#include "registration_apps/visualizer_ros.hpp"

namespace aicp {
class AppROS : public App {
public:
    AppROS(ros::NodeHandle& nh,
           const CommandLineConfig& cl_cfg,
           const ScanAccumulatorConfig& sa_cfg,
           const RegistrationParams& reg_params,
           const OverlapParams& overlap_params,
           const ClassificationParams& class_params,
           const string& bot_param_path);
    ~AppROS(){
    }

    void lidarScanCallBack(const sensor_msgs::LaserScan::ConstPtr& lidar_msg);
    void robotPoseCallBack(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_msg);
    void run();

private:
    ros::NodeHandle& nh_;
    tf::TransformListener body_to_lidar_listener_;

    tf::StampedTransform temp_tf_transf_;
    Eigen::Isometry3d temp_eigen_transf_;
    tf::Pose temp_tf_pose_;

    ros::Publisher pose_debug_pub_;
    ros::Publisher overlap_pub_;
    ros::Publisher risk_pub_;
    ros::Publisher align_pub_;

    LaserScanAccumulatorROS accu_;
    ScanAccumulatorConfig accu_config_;

    // Tool functions
    void getPoseAsIsometry3d(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose_msg,
                             Eigen::Isometry3d& eigen_pose);

};
} // namespace aicp
