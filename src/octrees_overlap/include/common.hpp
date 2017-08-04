#ifndef AICP_OVERLAP_COMMON_HPP_
#define AICP_OVERLAP_COMMON_HPP_

#include <iostream>
#include <string>

struct OverlapParams {
  std::string type;
  std::string loadPosesFromFile;

  struct OctreeOverlapParams {
    double octomapResolution;
  } octree_based;
};

#endif
