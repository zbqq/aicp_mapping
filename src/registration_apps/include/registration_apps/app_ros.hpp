#pragma once

#include <ros/node_handle.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "registration_apps/app.hpp"
#include "registration_apps/velodyne_accumulator.hpp"
#include "registration_apps/visualizer_ros.hpp"

namespace aicp {
class AppROS : public App {
public:
    AppROS(ros::NodeHandle& nh,
           const CommandLineConfig& cl_cfg,
           const VelodyneAccumulatorConfig& va_cfg,
           const RegistrationParams& reg_params,
           const OverlapParams& overlap_params,
           const ClassificationParams& class_params,
           const string& bot_param_path);
    ~AppROS(){
    }

    void velodyneCallBack(const sensor_msgs::PointCloud2::ConstPtr &laser_msg_in);
    void robotPoseCallBack(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_msg_in);
    void run();

private:
    ros::NodeHandle& nh_;
    tf::TransformListener body_to_lidar_listener_;

    tf::StampedTransform temp_tf_transf_;
    Eigen::Isometry3d temp_eigen_transf_;
    tf::Pose temp_tf_pose_;

    ros::Publisher corrected_pose_pub_;
    ros::Publisher overlap_pub_;
    ros::Publisher alignability_pub_;
    ros::Publisher risk_pub_;

//    VelodyneAccumulatorROS accu_;
    VelodyneAccumulatorConfig accu_config_;

    // Tool functions
    void getPoseAsIsometry3d(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose_msg,
                             Eigen::Isometry3d& eigen_pose);

};
} // namespace aicp
