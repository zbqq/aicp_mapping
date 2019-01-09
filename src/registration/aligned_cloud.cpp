#include "aicp_registration/aligned_cloud.hpp"

namespace aicp {

AlignedCloud::AlignedCloud(int64_t utime,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                           Eigen::Isometry3d prior_pose)
{ 
    utime_ = utime;
    cloud_ = cloud;

    world_to_cloud_prior_ = prior_pose;                         // prior pose
    cloud_to_reference_ = Eigen::Isometry3d::Identity();        // correction
    world_to_cloud_corrected_ = world_to_cloud_prior_;          // corrected pose (set equal to prior pose when correction not available yet)

    is_reference_ = false;  // default false
    its_reference_id_ = -1; // default -1 (indicates no alignment performed)
}

AlignedCloud::~AlignedCloud()
{ 
}

void AlignedCloud::updateCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                               bool is_reference)
{
    updateCloud(cloud, cloud_to_reference_, is_reference, its_reference_id_);
}
void AlignedCloud::updateCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                               Eigen::Isometry3d correction,
                               bool is_reference,
                               int its_reference_id)
{
    cloud_ = cloud;
    cloud_to_reference_ = correction;
    world_to_cloud_corrected_ = cloud_to_reference_ * world_to_cloud_prior_;
    is_reference_ = is_reference;
    its_reference_id_ = its_reference_id;
}
}

//void AlignedCloud::populateSweepScan(std::vector<LidarScan>& scans, pcl::PointCloud<pcl::PointXYZ> &cloud, int id, int refId, bool enRef)
//{
//  cloud_id = id;
//  this_cloud_ = cloud;

//  enabled_reference_ = enRef;
//  its_reference_id_ = refId;
  
//  planar_scans.assign(scans.begin(), scans.end());

//  utime_start = planar_scans.front().getUtime(); // Time stamp of first planar scan
//  utime_end = planar_scans.back().getUtime(); // Time stamp of last planar scan

//  Eigen::Isometry3d world_to_first_scan = planar_scans.front().getSensorPose();
//  world_to_last_scan = planar_scans.back().getSensorPose();
//  relative_motion = world_to_first_scan.inverse() * world_to_last_scan;

//  world_to_body = planar_scans.back().getBodyPose();

//  initialized_ = true;
//}

//pcl::PointCloud<pcl::PointXYZ>::Ptr AlignedCloud::getCloud()
//{
//  pcl::PointCloud<pcl::PointXYZ>::Ptr this_cloud_ptr;
//  this_cloud_ptr = this_cloud_.makeShared();

//  return this_cloud_ptr;
//}

//void AlignedCloud::addPointsToSweepScan(pcl::PointCloud<pcl::PointXYZ>& other_cloud)
//{
//  this_cloud_ += other_cloud;
//  this_cloud_.width = this_cloud_.points.size();
//  this_cloud_.height = 1;
//}

//void AlignedCloud::resetSweepScan()
//{
//  cloud_id = -1;
//  is_reference_ = false;
//  its_reference_id_ = -1;
//  this_cloud_.clear();

//  planar_scans.clear();

//  utime_start = -1;
//  utime_end = -1;

//  world_to_last_scan = Eigen::Isometry3d::Identity();
//  relative_motion = Eigen::Isometry3d::Identity();
//}
