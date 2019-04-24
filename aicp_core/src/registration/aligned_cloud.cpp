#include "aicp_registration/aligned_cloud.hpp"
#include "aicp_utils/common.hpp"

namespace aicp {

AlignedCloud::AlignedCloud(int64_t utime,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                           Eigen::Isometry3d prior_pose)
{ 
    utime_ = utime;
    cloud_ = cloud;

    world_to_cloud_odom_ = prior_pose;                          // prior pose (not updated else where)

    world_to_cloud_prior_ = prior_pose;                         // prior pose
    cloud_to_reference_ = Eigen::Isometry3d::Identity();        // correction
    world_to_cloud_corrected_ = world_to_cloud_prior_;          // corrected pose (set equal to prior pose when correction not available yet)

    is_reference_ = false;  // default false
    its_reference_id_ = -1; // default -1 (indicates no alignment performed)
}

AlignedCloud::~AlignedCloud()
{ 
}


// Take the roll and pitch from the odom and use it to replace the icp estimate roll and pitch
// this is to ensure gravity consistency
// TODO: the cloud_to_reference_ is now inconsistent with these two frames.
void AlignedCloud::removePitchRollCorrection()
{
    Eigen::Quaterniond q_odom = Eigen::Quaterniond(world_to_cloud_odom_.rotation());
    double r_odom, p_odom, y_odom;
    quat_to_euler(q_odom, r_odom, p_odom, y_odom);

    Eigen::Quaterniond q_corr = Eigen::Quaterniond(world_to_cloud_corrected_.rotation());
    double r_corr, p_corr, y_corr;
    quat_to_euler(q_corr, r_corr, p_corr, y_corr);

    //std::cout << r_odom*180/M_PI << " " 
    //          << p_odom*180/M_PI << " " 
    //          << y_odom*180/M_PI << " odom\n";
    //std::cout << r_corr*180/M_PI << " " 
    //          << p_corr*180/M_PI << " " 
    //          << y_corr*180/M_PI << " corr\n";

    Eigen::Isometry3d world_to_cloud_corrected_fixed = Eigen::Isometry3d::Identity();
    world_to_cloud_corrected_fixed.translation() = world_to_cloud_corrected_.translation();
    world_to_cloud_corrected_fixed.rotate( euler_to_quat(r_odom, p_odom, y_corr) );
    world_to_cloud_corrected_ = world_to_cloud_corrected_fixed;
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

    removePitchRollCorrection();

    is_reference_ = is_reference;
    its_reference_id_ = its_reference_id;
}
}
