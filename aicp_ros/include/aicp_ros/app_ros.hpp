#pragma once

#include <ros/node_handle.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <aicp_srv/ProcessFile.h>

#include "aicp_registration/app.hpp"
#include "velodyne_accumulator.hpp"
#include "visualizer_ros.hpp"

namespace aicp {
class AppROS : public App {
public:
    AppROS(ros::NodeHandle& nh,
           const CommandLineConfig& cl_cfg,
           const VelodyneAccumulatorConfig& va_cfg,
           const RegistrationParams& reg_params,
           const OverlapParams& overlap_params,
           const ClassificationParams& class_params);

    inline ~AppROS() {
        delete accu_;
        delete vis_ros_;
    }

    // Subscriber callabacks
    void velodyneCallBack(const sensor_msgs::PointCloud2::ConstPtr& laser_msg_in);
    void robotPoseCallBack(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_msg_in);
    void interactionMarkerCallBack(const geometry_msgs::PoseStampedConstPtr& init_pose_msg_in);

    // Advertise services
    bool loadMapFromFileCallBack(aicp_srv::ProcessFile::Request& request, aicp_srv::ProcessFile::Response& response);
    bool loadMapFromFile(const std::string& file_path);

    void run();

private:
    ros::NodeHandle& nh_;

    Eigen::Isometry3d world_to_body_;
    Eigen::Isometry3d world_to_body_previous_;
    tf::Pose temp_tf_pose_;

    ros::Publisher corrected_pose_pub_;
    ros::Publisher overlap_pub_;
    ros::Publisher alignability_pub_;
    ros::Publisher risk_pub_;

    VelodyneAccumulatorROS* accu_;
    VelodyneAccumulatorConfig accu_config_;

    // ROS only visualizer
    ROSVisualizer* vis_ros_;

    // Tool functions
    void getPoseAsIsometry3d(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose_msg,
                             Eigen::Isometry3d& eigen_pose);
    void getPoseAsIsometry3d(const geometry_msgs::PoseStampedConstPtr &pose_msg,
                             Eigen::Isometry3d& eigen_pose);

};
} // namespace aicp
