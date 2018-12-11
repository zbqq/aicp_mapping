#pragma once

#include <vector>
#include <Eigen/Geometry>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "aligned_cloud.hpp"

namespace aicp {

typedef std::shared_ptr<AlignedCloud> AlignedCloudPtr;

// Graph of aligned clouds
class AlignedCloudsGraph
{
public:
    AlignedCloudsGraph();
    ~AlignedCloudsGraph();

    void initialize(AlignedCloudPtr& reference); // global coordinates

    //    void updateReference(int index){ current_reference_ = aligned_clouds.at(index).getId(); };

    bool isEmpty(){ return !initialized_; }

    // Getters clouds
    int getNbClouds(){ return aligned_clouds.size(); }

    AlignedCloudPtr getLastCloud(){ return aligned_clouds.back(); }
    AlignedCloudPtr getCloudAt(int index){ return aligned_clouds.at(index); }
    std::vector<AlignedCloudPtr> getClouds(){ return aligned_clouds; }

    AlignedCloudPtr getCurrentReference(){ return aligned_clouds.at(current_reference_); }
    int getCurrentReferenceId(){ return current_reference_; }
bool addCloud(AlignedCloudPtr& cloud, Eigen::Isometry3d relative_pose, Eigen::Isometry3d pose, int reference_id);

private:
    bool initialized_;
    int current_reference_;

    std::vector<AlignedCloudPtr> aligned_clouds;  // Clouds in global reference frame (aligned)
};
}
