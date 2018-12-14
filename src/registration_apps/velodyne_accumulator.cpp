#include "registration_apps/velodyne_accumulator.hpp"
#include <pcl/point_types.h>

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


void VelodyneAccumulatorROS::processLidar(const sensor_msgs::PointCloud2::ConstPtr &cloud_in)
{
//    if(finished_){
//        ROS_WARN("LaserScanAccumulatorROS has finished accumulating.");
//        ROS_WARN_STREAM("Dropping scan " << scan_in->header.seq);
//        return;
//    }
//    scan_msg_ = *scan_in;
//    if(!listener_.waitForTransform(
//            scan_msg_.header.frame_id,
//            config_.inertial_frame, // put the point cloud into an inertial frame
//            scan_msg_.header.stamp + ros::Duration().fromSec(scan_msg_.ranges.size()*scan_msg_.time_increment),
//            ros::Duration(1.0))){
//         return;
//      }
//    std::vector<float>::iterator it = scan_msg_.ranges.begin();

//    for(; it != scan_msg_.ranges.end(); ++it){
//        // artificially setting the ranges below the minimum range to
//        // above the maximum range so the projector will cut it off
//        if(*it < config_.min_range)
//        {
//            *it = config_.max_range + 1000;
//        }
//    }

//      projector_.transformLaserScanToPointCloud(config_.inertial_frame,
//                                                scan_msg_,
//                                                point_cloud_ros_msg_,
//                                                listener_,
//                                                config_.max_range,
//                                                laser_geometry::channel_option::Intensity);
//      pcl_conversions::toPCL(point_cloud_ros_msg_, point_cloud_pcl_msg_);
//      pcl::fromPCLPointCloud2<pcl::PointXYZ>(point_cloud_pcl_msg_,point_cloud_);
//      accumulated_point_cloud_ += point_cloud_;

//      if(++counter >= config_.batch_size){
//          finished_ = true;
//          utime_ = scan_in->header.stamp.toNSec() / 1000;
//      }
}

void VelodyneAccumulatorROS::clearCloud(){
    point_cloud_.clear();
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
