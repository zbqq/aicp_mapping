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
