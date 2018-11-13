#include "aicp_registration/sweep_scan.hpp"

SweepScan::SweepScan()
{ 
  cloud_id = -1;

  is_reference_ = false;
  enabled_reference_ = true;
  its_reference_id_ = -1;
  
  planar_scans.clear();

  utime_start = -1;
  utime_end = -1;

  world_to_last_scan = Eigen::Isometry3d::Identity();
  relative_motion = Eigen::Isometry3d::Identity();

  initialized_ = false;
}

SweepScan::~SweepScan()
{ 
}

// Called to initialize first reference cloud only
void SweepScan::populateSweepScan(std::vector<LidarScan>& scans, pcl::PointCloud<pcl::PointXYZI> &cloud, int id)
{
  populateSweepScan(scans, cloud, id, -1, true);
  is_reference_ = true;
}

void SweepScan::populateSweepScan(std::vector<LidarScan>& scans, pcl::PointCloud<pcl::PointXYZI> &cloud, int id, int refId, bool enRef)
{
  cloud_id = id;
  this_cloud_ = cloud;

  enabled_reference_ = enRef;
  its_reference_id_ = refId;
  
  planar_scans.assign(scans.begin(), scans.end());

  utime_start = planar_scans.front().getUtime(); // Time stamp of first planar scan
  utime_end = planar_scans.back().getUtime(); // Time stamp of last planar scan

  Eigen::Isometry3d world_to_first_scan = planar_scans.front().getSensorPose();
  world_to_last_scan = planar_scans.back().getSensorPose();
  relative_motion = world_to_first_scan.inverse() * world_to_last_scan;

  world_to_body = planar_scans.back().getBodyPose();

  initialized_ = true;
}

void SweepScan::addPointsToSweepScan(pcl::PointCloud<pcl::PointXYZI>& other_cloud)
{
  this_cloud_ += other_cloud;
  this_cloud_.width = this_cloud_.points.size();
  this_cloud_.height = 1;
}

void SweepScan::resetSweepScan()
{
  cloud_id = -1;
  is_reference_ = false;
  its_reference_id_ = -1;
  this_cloud_.clear();

  planar_scans.clear();

  utime_start = -1;
  utime_end = -1;

  world_to_last_scan = Eigen::Isometry3d::Identity();
  relative_motion = Eigen::Isometry3d::Identity();
}

pcl::PointCloud<pcl::PointXYZI>::Ptr SweepScan::getCloud()
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr this_cloud_ptr;
  this_cloud_ptr = this_cloud_.makeShared();

  return this_cloud_ptr;
}
