#include "registration_apps/velodyne_accumulator.hpp"

#include <pcl/point_types.h>

using namespace std;
using PointCloud = aicp::VelodyneAccumulatorROS::PointCloud;

namespace aicp {

VelodyneAccumulatorROS::VelodyneAccumulatorROS(ros::NodeHandle &nh,
                                               const VelodyneAccumulatorConfig &config) :
                                               nh_(nh), config_(config)
{
//    lidar_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(config_.lidar_topic,
//                                                         100,
//                                                         &VelodyneAccumulatorROS::processLidar,
//                                                         this);
}

void VelodyneAccumulatorROS::setConfig(const VelodyneAccumulatorConfig &config){
    config_ = config;
    lidar_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(config_.lidar_topic,
                                                         100,
                                                         &VelodyneAccumulatorROS::processLidar,
                                                         this);
}


void VelodyneAccumulatorROS::processLidar(const sensor_msgs::PointCloud2::ConstPtr& cloud_in)
{
    if(finished_){
        return;
    }
    cloud_msg_ = *cloud_in;

    ros::Time msg_time(cloud_msg_.header.stamp.sec, cloud_msg_.header.stamp.nsec);
    tf::StampedTransform body_pose_tf;
    try {
        // waitForTransform( to frame, from frame, ... )
        listener_.waitForTransform(config_.inertial_frame, cloud_msg_.header.frame_id, msg_time, ros::Duration(1.0));
        listener_.lookupTransform(config_.inertial_frame, cloud_msg_.header.frame_id, msg_time, body_pose_tf);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s : ", ex.what());
        ROS_ERROR("Skipping point cloud.");
        return;
    }
    Eigen::Isometry3d body_pose_eigen;
    tf::transformTFToEigen(body_pose_tf, body_pose_eigen);

    // Transform point cloud to global frame
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_in,*cloud_tmp);
    pcl::transformPointCloud(*cloud_tmp, point_cloud_, body_pose_eigen.translation().cast<float>(),
                             Eigen::Quaternionf(body_pose_eigen.rotation().cast<float>()));

    // Accumulate
    accumulated_point_cloud_ += point_cloud_;
    utime_ = cloud_msg_.header.stamp.toNSec() / 1000;

    // Check number of accumulated clouds
    if(++counter >= config_.batch_size){
        finished_ = true;
    }
}

void VelodyneAccumulatorROS::clearCloud(){
    point_cloud_.clear();
    accumulated_point_cloud_.clear();
    finished_ = false;
    counter = 0;
}

const PointCloud& VelodyneAccumulatorROS::getCloud(){
    return accumulated_point_cloud_;
}

uint16_t VelodyneAccumulatorROS::getCounter() const{
    return counter;
}

bool VelodyneAccumulatorROS::getFinished() const {
    return finished_;
}

uint64_t VelodyneAccumulatorROS::getFinishedTime() const{
    return utime_;
}
} // namespace aicp
