#include "aligned_sweeps_collection.hpp"

AlignedSweepsCollection::AlignedSweepsCollection()
{ 
  initialized_ = false;
  last_reference_ = -1;
}

AlignedSweepsCollection::~AlignedSweepsCollection()
{ 
}

void AlignedSweepsCollection::initializeCollection(SweepScan reference)
{
  aligned_clouds.push_back(reference);
  last_reference_ = reference.getId();

  utime_start = aligned_clouds.front().getUtimeEnd();
  utime_end = aligned_clouds.back().getUtimeEnd();

  reference_to_current_cloud = Eigen::Matrix4f::Identity(4,4);

  initialized_ = true;
}

void AlignedSweepsCollection::addSweep(SweepScan current_aligned, Eigen::Matrix4f T_ref_curr)
{
  aligned_clouds.push_back(current_aligned);

  utime_end = aligned_clouds.back().getUtimeEnd();

  reference_to_current_cloud = T_ref_curr;
}
