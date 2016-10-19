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
    bool blur_map;
};


class ConvertOctomap{
  public:
    ConvertOctomap(boost::shared_ptr<lcm::LCM> &lcm_, const ConvertOctomapConfig& co_cfg_);
    
    ~ConvertOctomap(){
    }    

    void doWork(pcl::PointCloud<pcl::PointXYZRGB> &cloud, string octree_channel);
    ColorOcTree* getTree(){ return tree_; }
    bool clearTree(){ 
      tree_->clear();
      return true; 
    }
    void publishOctree(ColorOcTree* tree, string octree_channel);
    void printChangesByColor(ColorOcTree& tree);
    void printChangesAndActual(ColorOcTree& tree);
    void colorChanges(ColorOcTree& tree, int idx); //idx is an index representing 
                                                   //the current cloud, either 0 or 1
    
  private:

    boost::shared_ptr<lcm::LCM> lcm_;

    ColorOcTree* tree_;    
    
    const ConvertOctomapConfig co_cfg_;   
    
    int verbose_;

    //Colors for change detection
    ColorOcTreeNode::Color* yellow;
    ColorOcTreeNode::Color* blue;
    ColorOcTreeNode::Color* green;

    void updateOctree(pcl::PointCloud<pcl::PointXYZRGB> &cloud, ColorOcTree* tree);
    ScanGraph* convertPointCloudToScanGraph(pcl::PointCloud<pcl::PointXYZRGB> &cloud);
};


#endif
