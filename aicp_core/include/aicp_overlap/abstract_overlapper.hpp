#ifndef AICP_OCTREES_OVERLAP_ABSTRACT_HPP_
#define AICP_OCTREES_OVERLAP_ABSTRACT_HPP_

#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>

#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>

using namespace octomap;

namespace aicp {
  class AbstractOverlapper {
  public:
    virtual ColorOcTree* computeOverlap(pcl::PointCloud<pcl::PointXYZ> &ref_cloud, pcl::PointCloud<pcl::PointXYZ> &read_cloud,
                                        Eigen::Isometry3d ref_pose, Eigen::Isometry3d read_pose,
                                        ColorOcTree* reading_tree) = 0;
    virtual float getOverlap() = 0;
  };
}

#endif
