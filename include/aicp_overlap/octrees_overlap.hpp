#ifndef AICP_OCTREES_OVERLAP_HPP_
#define AICP_OCTREES_OVERLAP_HPP_

#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>

#include <pcl/point_types.h>
#include <pcl/common/io.h>

//Project lib
#include "aicp_overlap/common.hpp"
#include "aicp_overlap/abstract_overlapper.hpp"


using namespace std;
using namespace octomap;

namespace aicp{

class OctreesOverlap : public AbstractOverlapper {
  public:
    OctreesOverlap();
    explicit OctreesOverlap(const OverlapParams& params);
    ~OctreesOverlap();

    virtual ColorOcTree* computeOverlap(pcl::PointCloud<pcl::PointXYZ> &ref_cloud, pcl::PointCloud<pcl::PointXYZ> &read_cloud,
                                        Eigen::Isometry3d ref_pose, Eigen::Isometry3d read_pose,
                                        ColorOcTree* reading_tree);

    virtual float getOverlap(){ return overlap_; }
//    ColorOcTree* getTree(){ return tree_; }
//    bool clearTree(){
//      tree_->clear();
//      return true;
//    }

    // TODO -----------------------------------------
    float computeLoopClosureFromOverlap(ColorOcTree* treeA, ColorOcTree* treeB);
    // TODO -----------------------------------------
    
  private:
    OverlapParams params_;

    ColorOcTree* tree_; // Octree created from reference cloud

    float overlap_;

    //Colors for change detection
    ColorOcTreeNode::Color* yellow; //old occupied
    ColorOcTreeNode::Color* blue;   //always occupied
    ColorOcTreeNode::Color* green;  //new occupied
    ColorOcTreeNode::Color* red;

    void getOverlappingNodes(ColorOcTree* treeA, ColorOcTree* treeB, int& overlapping_nodes, int& count_nodes_ref, int& count_nodes_read);
    void createTree(pcl::PointCloud<pcl::PointXYZ> &cloud, Eigen::Isometry3d pose, ColorOcTree* output_tree, ColorOcTreeNode::Color* color);
    void setReferenceTree(pcl::PointCloud<pcl::PointXYZ> &ref_cloud, Eigen::Isometry3d ref_pose);
    ScanGraph* convertPointCloudToScanGraph(pcl::PointCloud<pcl::PointXYZ> &cloud, Eigen::Isometry3d sensor_pose);
};

}
#endif
