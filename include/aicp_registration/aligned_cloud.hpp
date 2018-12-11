#pragma once

#include <vector>
#include <Eigen/Geometry>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace aicp {

// Point cloud class
class AlignedCloud
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    AlignedCloud(long long int utime,
                 pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                 Eigen::Isometry3d prior_pose);
    ~AlignedCloud();

    void updateCloud(Eigen::Isometry3d correction,
                     Eigen::Isometry3d corrected_pose,
                     bool is_reference,
                     int its_reference_id);

    // Getters
    long long int getUtime(){ return utime_; }

    pcl::PointCloud<pcl::PointXYZ>::Ptr getCloud(){ return cloud_; }
    int getNbPoints(){ return cloud_->size(); }

    Eigen::Isometry3d getPriorPose(){ return world_to_cloud_prior_; }
    Eigen::Isometry3d getCorrection(){ return cloud_to_reference_; }
    Eigen::Isometry3d getCorrectedPose(){ return world_to_cloud_corrected_; }

    int getItsReferenceId(){ return its_reference_id_; }

    // Setters
    void setReference(){ is_reference_ = true; }

    bool isReference(){ return is_reference_; }

private:
    long long int utime_; // Cloud timestamp (microseconds)

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_; // Cloud (global coordinates)

    Eigen::Isometry3d world_to_cloud_prior_;         // cloud pose prior:     world -> cloud (global coordinates)
    Eigen::Isometry3d cloud_to_reference_;           // relative transform:   cloud -> reference (global coordinates)
    Eigen::Isometry3d world_to_cloud_corrected_;     // cloud pose corrected: world -> cloud (global coordinates)

    bool is_reference_; // Is this cloud a reference cloud?

    // if scan is a reading cloud this field contains the id of the reference used for alignment
    // if scan is the first reference cloud this field is set to -1
    int its_reference_id_;
};
}
