#pragma once

#include <Eigen/Geometry>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace aicp {

// Point cloud class
class AlignedCloud
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    AlignedCloud(int64_t utime,
                 pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                 Eigen::Isometry3d prior_pose);
    ~AlignedCloud();

    void removePitchRollCorrection();

    void updateCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                     bool is_reference);
    void updateCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                     Eigen::Isometry3d correction,
                     bool is_reference,
                     int its_reference_id);

    // Getters
    int64_t getUtime(){ return utime_; }

    pcl::PointCloud<pcl::PointXYZ>::Ptr getCloud(){ return cloud_; }
    int getNbPoints(){ return cloud_->size(); }

    Eigen::Isometry3d getOdomPose(){ return world_to_cloud_odom_; }
    Eigen::Isometry3d getPriorPose(){ return world_to_cloud_prior_; }
    Eigen::Isometry3d getCorrection(){ return cloud_to_reference_; }
    Eigen::Isometry3d getCorrectedPose(){ return world_to_cloud_corrected_; }

    int getItsReferenceId(){ return its_reference_id_; }

    // Setters
    void setReference(){ is_reference_ = true; }

    void setPriorPose(Eigen::Isometry3d pose_prior)
    {
        world_to_cloud_prior_ = pose_prior;
        world_to_cloud_corrected_ = cloud_to_reference_ * world_to_cloud_prior_;
    }

    bool isReference(){ return is_reference_; }

private:
    int64_t utime_; // Cloud timestamp (microseconds)

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_; // Cloud (pre-filtered and global coordinates)

    Eigen::Isometry3d world_to_cloud_odom_;          // odom to base:         world -> cloud (global coordinates). this is the unmodified input.

    Eigen::Isometry3d world_to_cloud_prior_;         // cloud pose prior:     world -> cloud (global coordinates)
    Eigen::Isometry3d cloud_to_reference_;           // relative transform:   cloud -> reference (global coordinates)
    Eigen::Isometry3d world_to_cloud_corrected_;     // cloud pose corrected: world -> cloud (global coordinates)

    bool is_reference_; // Is this cloud a reference cloud?

    // if scan is a reading cloud this field contains the id of the reference used for alignment
    // if scan is the first reference cloud this field is set to -1
    int its_reference_id_;
};
}
