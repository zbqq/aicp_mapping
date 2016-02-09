#include "sweep_scan.hpp"

SweepScan::SweepScan()
{ 
}

SweepScan::~SweepScan()
{ 
}

void SweepScan::populateSweepScan(std::vector<LidarScan>& scans, DP& cloud, int id)
{
  cloud_id = id;
  dp_cloud = cloud;
  
  planar_scans.assign(scans.begin(), scans.end());

  utime_start = planar_scans.front().getUtime(); // Time stamp of first planar scan
  utime_start = planar_scans.back().getUtime(); // Time stamp of last planar scan

  Eigen::Isometry3d world_to_first = planar_scans.front().getPose();
  Eigen::Isometry3d world_to_last = planar_scans.back().getPose();
  relative_motion = world_to_first.inverse() * world_to_last;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr SweepScan::getPCLCloud()
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPCL (new pcl::PointCloud<pcl::PointXYZRGB> ());

  fromPCLToDataPoints(dp_cloud, *cloudPCL);

  return cloudPCL;
}