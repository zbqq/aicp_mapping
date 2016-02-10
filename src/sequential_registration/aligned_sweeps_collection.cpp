#include "aligned_sweeps_collection.hpp"

AlignedSweepsCollection::AlignedSweepsCollection()
{ 
  initialized_ = false;
}

AlignedSweepsCollection::~AlignedSweepsCollection()
{ 
}

void AlignedSweepsCollection::initializeCollection(SweepScan reference)
{
  aligned_clouds.push_back(reference);

  utime_start = aligned_clouds.front().getUtimeEnd(); 
  utime_end = aligned_clouds.back().getUtimeEnd();

  reference_to_current_cloud = PM::Matrix::Identity(4,4);

  initialized_ = true;
}

void AlignedSweepsCollection::addSweep(SweepScan current_aligned, PM::TransformationParameters T_ref_curr)
{
  aligned_clouds.push_back(current_aligned);

  utime_end = aligned_clouds.back().getUtimeEnd();

  reference_to_current_cloud = T_ref_curr;
}