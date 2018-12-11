#pragma once

#include <laser_geometry/laser_geometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>

namespace aicp {

struct ScanAccumulatorConfig
{
    int batch_size = 83;
    std::string lidar_topic;
    double max_range = 30;
    double min_range = 0.5;
    std::string inertial_frame = "/odom";
};

class LaserScanAccumulatorROS {
public:
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
public:
    LaserScanAccumulatorROS(ros::NodeHandle& nh,
                            const ScanAccumulatorConfig& config = ScanAccumulatorConfig());
    void processLidar(const sensor_msgs::LaserScan::ConstPtr& scan_in);
    // functions to mimick the same behavior of MIT's cloud accumulator class
    void clearCloud();
    uint16_t getCounter() const;
    bool getFinished() const;
    void setConfig(const ScanAccumulatorConfig& config);
    uint64_t getFinishedTime() const;
    const PointCloud& getCloud();

private:
    laser_geometry::LaserProjection projector_;
    tf::TransformListener listener_;
    ScanAccumulatorConfig config_;
    sensor_msgs::PointCloud2 point_cloud_ros_msg_;
    pcl::PCLPointCloud2 point_cloud_pcl_msg_;
    sensor_msgs::LaserScan scan_msg_;
    // implicitly discarding intensities from the scans
    PointCloud point_cloud_;
    PointCloud accumulated_point_cloud_;
    uint64_t utime_;

    bool finished_ = false;
    uint16_t counter = 0;
    ros::NodeHandle& nh_;
    ros::Subscriber lidar_sub_;
};
}
