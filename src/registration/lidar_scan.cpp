#include "aicp_registration/lidar_scan.hpp"

LidarScan::LidarScan(long long int t_stamp,float init_angle,float step_angle,
                     std::vector< float > r,std::vector< float > i,
                     Eigen::Isometry3d head_pose, Eigen::Isometry3d lidar_pose,
                     Eigen::Isometry3d body_pose)
{ 
  utime = t_stamp;
  rad0 = init_angle; 
  radstep = step_angle;
    
  ranges.assign (r.begin(),r.end());
    
  intensities.assign (i.begin(),i.end());  

  world_to_head = head_pose;
  head_to_lidar = lidar_pose;
  world_to_body = body_pose;
}

LidarScan::~LidarScan()
{ 
}