#pragma once

#include "aligned_cloud.hpp"

namespace aicp {

typedef std::shared_ptr<AlignedCloud> AlignedCloudPtr;

// Graph of aligned clouds
class AlignedCloudsGraph
{
public:
    AlignedCloudsGraph();
    ~AlignedCloudsGraph();

    void initialize(AlignedCloudPtr& reference);
    bool addCloud(AlignedCloudPtr& cloud);

    void updateReference(int index)
    {
        aligned_clouds.at(index)->setReference();
        current_reference_ = index;
    }

    bool isEmpty(){ return !initialized_; }

    // Getters clouds
    int getNbClouds(){ return aligned_clouds.size(); }

    AlignedCloudPtr getCloudAt(int index){ return aligned_clouds.at(index); }
    std::vector<AlignedCloudPtr> getClouds(){ return aligned_clouds; }

    AlignedCloudPtr getCurrentReference(){ return aligned_clouds.at(current_reference_); }
    int getCurrentReferenceId(){ return current_reference_; }
    AlignedCloudPtr getLastCloud(){ return aligned_clouds.back(); }
    int getLastCloudId(){ return aligned_clouds.size()-1; }

private:
    bool initialized_;
    int current_reference_;

    std::vector<AlignedCloudPtr> aligned_clouds;  // Clouds in global reference frame (aligned)
};
}
