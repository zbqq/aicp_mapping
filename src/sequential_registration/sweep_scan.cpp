#include "sweep_scan.hpp"

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
void SweepScan::populateSweepScan(std::vector<LidarScan>& scans, DP& cloud, int id)
{
  populateSweepScan(scans, cloud, id, -1, true);
  is_reference_ = true;
}

void SweepScan::populateSweepScan(std::vector<LidarScan>& scans, DP& cloud, int id, int refId, bool enRef)
{
  cloud_id = id;
  dp_cloud = cloud;

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

void cleanDataPoint(DP& cloud)
{
  cloud = cloud.createSimilarEmpty();
  /*
  // Clean point features
  if(cloud.featureExists("x"))
    cloud.removeFeature ("x");
  if(cloud.featureExists("y"))
    cloud.removeFeature ("y");
  if(cloud.featureExists("z"))
    cloud.removeFeature ("z");
  if(cloud.featureExists("pad"))
    cloud.removeFeature ("pad");

  // Clean point descriptors
  // Color
  if(cloud.descriptorExists("r"))
    cloud.removeDescriptor ("r");
  if(cloud.descriptorExists("g"))
    cloud.removeDescriptor ("g");
  if(cloud.descriptorExists("b"))
    cloud.removeDescriptor ("b");
  // Normal (DON'T KNOW HOW IT IS DEFINED --> CHANGE NAMES!)
  if(cloud.descriptorExists("x"))
    cloud.removeDescriptor ("x");
  if(cloud.descriptorExists("y"))
    cloud.removeDescriptor ("y");
  if(cloud.descriptorExists("z"))
    cloud.removeDescriptor ("z");
  */
}

void SweepScan::addPointsToSweepScan(DP other_cloud)
{
  dp_cloud.concatenate(other_cloud);
}

void SweepScan::resetSweepScan()
{
  cloud_id = -1;
  is_reference_ = false;
  its_reference_id_ = -1;
  cleanDataPoint(dp_cloud);
  
  planar_scans.clear();

  utime_start = -1;
  utime_end = -1;

  world_to_last_scan = Eigen::Isometry3d::Identity();
  relative_motion = Eigen::Isometry3d::Identity();
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr SweepScan::getPCLCloud()
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPCL (new pcl::PointCloud<pcl::PointXYZRGB> ());
  fromDataPointsToPCL(dp_cloud, *cloudPCL);

  return cloudPCL;
}