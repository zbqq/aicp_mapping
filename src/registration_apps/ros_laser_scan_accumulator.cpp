#include "registration_apps/ros_laser_scan_accumulator.hpp"
#include <pcl/point_types.h>

using PointCloud = aicp::LaserScanAccumulatorROS::PointCloud;

namespace aicp {

LaserScanAccumulatorROS::LaserScanAccumulatorROS(ros::NodeHandle &nh,
                                                 const ScanAccumulatorConfig &config) :
    nh_(nh)
{

}

void LaserScanAccumulatorROS::setConfig(const ScanAccumulatorConfig &config){
    config_ = config;
}

void LaserScanAccumulatorROS::clearCloud(){
    point_cloud_.clear();
    finished_ = false;
    counter = 0;
}

const PointCloud& LaserScanAccumulatorROS::getCloud(){
    return accumulated_point_cloud_;
}

void LaserScanAccumulatorROS::processLidar(const sensor_msgs::LaserScan::ConstPtr &scan_in)
{
    if(finished_){
        ROS_WARN("LaserScanAccumulatorROS has finished accumulating.");
        ROS_WARN_STREAM("Dropping scan " << scan_in->header.seq);
        return;
    }
    scan_msg_ = *scan_in;
    if(!listener_.waitForTransform(
            scan_msg_.header.frame_id,
            config_.inertial_frame, // put the point cloud into an inertial frame
            scan_msg_.header.stamp + ros::Duration().fromSec(scan_msg_.ranges.size()*scan_msg_.time_increment),
            ros::Duration(1.0))){
         return;
      }
    std::vector<float>::iterator it = scan_msg_.ranges.begin();

    for(; it != scan_msg_.ranges.end(); ++it){
        // artificially setting the ranges below the minimum range to
        // above the maximum range so the projector will cut it off
        if(*it < config_.min_range)
        {
            *it = config_.max_range + 1000;
        }
    }

      projector_.transformLaserScanToPointCloud(config_.inertial_frame,
                                                scan_msg_,
                                                point_cloud_ros_msg_,
                                                listener_,
                                                config_.max_range,
                                                laser_geometry::channel_option::Intensity);
      pcl_conversions::toPCL(point_cloud_ros_msg_, point_cloud_pcl_msg_);
      pcl::fromPCLPointCloud2<pcl::PointXYZI>(point_cloud_pcl_msg_,point_cloud_);
      accumulated_point_cloud_ += point_cloud_;

      if(++counter >= config_.batch_size){
          finished_ = true;
          utime_ = scan_in->header.stamp.toNSec() / 1000;
      }
}

uint16_t LaserScanAccumulatorROS::getCounter() const{
    return counter;
}

bool LaserScanAccumulatorROS::getFinished() const {
    return finished_;
}

uint64_t LaserScanAccumulatorROS::getFinishedTime() const{
    return utime_;
}
} // namespace aicp
