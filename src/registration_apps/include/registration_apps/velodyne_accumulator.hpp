#pragma once

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>

namespace aicp {

struct VelodyneAccumulatorConfig
{
    int batch_size = 80;
    double max_range = 30;
    double min_range = 0.5;
    std::string lidar_topic = "/velodyne/point_cloud_filtered";
    std::string inertial_frame = "/odom";
};

class VelodyneAccumulatorROS {
public:
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
public:
    VelodyneAccumulatorROS(ros::NodeHandle& nh,
                           const VelodyneAccumulatorConfig& config = VelodyneAccumulatorConfig());

    void processLidar(const sensor_msgs::PointCloud2::ConstPtr& cloud_in);

    // functions to mimick the same behavior of MIT's cloud accumulator class
    void setConfig(const VelodyneAccumulatorConfig& config);

    uint16_t getCounter() const;
    bool getFinished() const;
    uint64_t getFinishedTime() const;

    const PointCloud& getCloud();
    void clearCloud();

private:
    tf::TransformListener listener_;
    VelodyneAccumulatorConfig config_;
    laser_geometry::LaserProjection projector_;

    sensor_msgs::PointCloud2 point_cloud_ros_msg_;
    sensor_msgs::PointCloud2 scan_msg_;
//    pcl::PCLPointCloud2 point_cloud_pcl_msg_;

    // implicitly discarding intensities from the scans
    PointCloud point_cloud_;
    PointCloud accumulated_point_cloud_;
    uint64_t utime_;

    ros::NodeHandle& nh_;
    ros::Subscriber lidar_sub_;
    bool finished_ = false;
    uint16_t counter = 0;
};
}
