#pragma once
#include "registration_apps/app.hpp"
#include <ros/node_handle.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>

namespace aicp {
class AppROS : public App {
public:
    AppROS(ros::NodeHandle& nh,
           const CommandLineConfig& cl_cfg,
           CloudAccumulateConfig ca_cfg,
           RegistrationParams reg_params,
           OverlapParams overlap_params,
           ClassificationParams class_params,
           string exp_params);
    ~AppROS(){

    }

    void lidarScanCallBack(const sensor_msgs::LaserScan::ConstPtr& lidar_msg);
    void robotPoseCallBack(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_msg);
    void run();
private:
    ros::NodeHandle& nh_;
    tf::TransformListener body_to_lidar_listener_;
    pronto_vis* pc_vis_;
    CloudAccumulateConfig ca_cfg_;

};
} // namespace aicp
