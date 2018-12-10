#pragma once

#include <vector>
#include <Eigen/Geometry>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "aligned_cloud.hpp"

// Graph of aligned clouds
class AlignedCloudsGraph
{
  public:
    AlignedCloudsGraph();
    ~AlignedCloudsGraph();

    void initialize(AlignedCloud& reference); // global coordinates
    bool addCloud(AlignedCloud& cloud, Eigen::Isometry3d relative_pose, Eigen::Isometry3d pose, int reference_id);

//    void updateReference(int index){ current_reference_ = aligned_clouds.at(index).getId(); };

    bool isEmpty(){ return !initialized_; };

    // Getters clouds
    int getNbClouds(){ return aligned_clouds.size(); };

    AlignedCloud& getLastCloud(){ return aligned_clouds.back(); };
    AlignedCloud& getCloudAt(int index){ return aligned_clouds.at(index); };
    std::vector<AlignedCloud>& getClouds(){ return aligned_clouds; };

//    Eigen::Isometry3d getLastPose(); // global cloud_to_world pose (aligned)
//    Eigen::Isometry3d getPoseAt();   // global cloud_to_world pose (aligned)

//    Eigen::Isometry3d getLastRelativePose(); // global cloud_to_reference pose (aligned)
//    Eigen::Isometry3d getRelativePoseAt();   // global cloud_to_reference pose (aligned)

    AlignedCloud& getCurrentReference(){ return aligned_clouds.at(current_reference_); };
    int getCurrentReferenceId(){ return current_reference_; };


  private:
    bool initialized_;
    int current_reference_;

    std::vector<AlignedCloud> aligned_clouds;  // Clouds in global reference frame (aligned)

//    Eigen::Isometry3d reference_to_current_cloud; // Current pose with respect to reference cloud
//                                                  // constraint after drift correction
};
