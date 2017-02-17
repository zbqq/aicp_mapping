#ifndef CONVERT_OCTOMAP_HPP_
#define CONVERT_OCTOMAP_HPP_

#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap_utils/octomap_util.hpp>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <lcmtypes/octomap_utils.h>

using namespace std;
using namespace octomap;

struct ConvertOctomapConfig
{
    double octomap_resolution;
    double blur_sigma;
};


class ConvertOctomap{
  public:
    ConvertOctomap(boost::shared_ptr<lcm::LCM> &lcm_, const ConvertOctomapConfig& co_cfg_);
    
    ~ConvertOctomap(){
    }    

    void doConversion(pcl::PointCloud<pcl::PointXYZRGB> &cloud_in, int cloud_idx); //cloud indexes are 0 if reference, 1 if reading.
    void createBlurredOctree(ColorOcTree* tree);

    ColorOcTree* getTree(){ return tree_; }
    bool clearTree(){ 
      tree_->clear();
      return true; 
    }
    void publishOctree(ColorOcTree* tree, string octree_channel);
    void publishOctree(OcTree* tree, string octree_channel);
    void printChangesByColor(ColorOcTree& tree);
    void printChangesAndActual(ColorOcTree& tree);
    void colorChanges(ColorOcTree& tree, int idx); //idx is an index representing 
                                                   //the current cloud, either 0 or 1
    
  private:

    boost::shared_ptr<lcm::LCM> lcm_;
    const ConvertOctomapConfig co_cfg_;

    ColorOcTree* tree_;       

    //Colors for change detection
    ColorOcTreeNode::Color* yellow; //old occupied
    ColorOcTreeNode::Color* blue;   //always occupied
    ColorOcTreeNode::Color* green;  //new occupied
    ColorOcTreeNode::Color* red;

    void updateOctree(pcl::PointCloud<pcl::PointXYZRGB> &cloud, ColorOcTree* tree);
    ScanGraph* convertPointCloudToScanGraph(pcl::PointCloud<pcl::PointXYZRGB> &cloud);
};


#endif
