// Get 2 point clouds (default).
// Convert to first cloud to octree and publish to lcm.
// Update the octree with the second cloud (octrees difference with color coding):
//      blue - occupied voxels in both octrees
//      yellow - old occupied voxels
//      green - new occupied voxels
// Republish the octree.

// RUN: octrees-difference -h

#include <sstream>      // std::stringstream

#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

#include "pointmatcher/PointMatcher.h"
#include <icp-registration/icp_utils.h>

#include <octrees-difference/convert_octomap.hpp>

//Compare
#include <octomap/octomap.h>
#include <octomap/math/Utils.h>

#include "octomap_utils/octomap_util.hpp"

//Help
#include <ConciseArgs>

struct AppConfig
{
  string cloud1;
  string cloud2;
};

class App{
  public:
    App(boost::shared_ptr<lcm::LCM> &lcm_, 
        ConvertOctomapConfig co_cfg_, AppConfig app_cfg);
    
    ~App(){
    }  

    ConvertOctomapConfig co_cfg_;
    AppConfig app_cfg_;
    
    boost::shared_ptr<lcm::LCM> lcm_;
    ConvertOctomap* convert_;

    // Clouds
    string cloudA_name_;
    string cloudB_name_;

    bool do_republish_; // true if an octree is ready  

  private:

};

App::App(boost::shared_ptr< lcm::LCM >& lcm_, 
         ConvertOctomapConfig co_cfg_, AppConfig app_cfg_) : lcm_(lcm_), 
         co_cfg_(co_cfg_),
         app_cfg_(app_cfg_){

  convert_ = new ConvertOctomap(lcm_, co_cfg_); 
  std::cout << "Clouds to octrees conversion at launch.\n";

  do_republish_ = false;

  if (app_cfg_.cloud1.empty() || app_cfg_.cloud2.empty())
  {
    cloudA_name_.append(getenv("DRC_BASE"));
    cloudB_name_.append(getenv("DRC_BASE"));
    cloudA_name_.append("/software/registration/data/cloud_01.vtk");
    cloudB_name_.append("/software/registration/data/cloud_02.vtk");
  }
  else
  {
    cloudA_name_.append(app_cfg_.cloud1);
    cloudB_name_.append(app_cfg_.cloud2);    
  }

  // Load point clouds from file
  DP cloudA = DP::load(cloudA_name_);
  DP cloudB = DP::load(cloudB_name_);

  //===========================================
  // CONVERT CLOUD TO OCTREE AND DIFFERENTIATE
  //=========================================== 
  
  pcl::PointCloud<pcl::PointXYZRGB> pcl_cloudA;
  fromDataPointsToPCL(cloudA, pcl_cloudA);
  ColorOcTree* treeA;

  convert_->doConversion(pcl_cloudA, 0);
  treeA = convert_->getTree();
  cout << "Pruned tree A size: " << treeA->size() <<" nodes." << endl;
  cout << "Changes A:" << endl;
  convert_->printChangesAndActual(*treeA);
  convert_->publishOctree(treeA, "OCTOMAP_REF");

  // Uncomment for visualization of matching results one by one:
  cout << "Press ENTER to continue..." << endl;
  cin.get();

  pcl::PointCloud<pcl::PointXYZRGB> pcl_cloudB;
  fromDataPointsToPCL(cloudB, pcl_cloudB);
  ColorOcTree* treeB;
    
  convert_->doConversion(pcl_cloudB, 1);
  treeB = convert_->getTree();
  cout << "Pruned tree B size: " << treeB->size() <<" nodes." << endl;
  cout << "Changes B:" << endl;
  convert_->printChangesByColor(*treeB);
  convert_->publishOctree(treeB, "OCTOMAP_IN");

  //===========================================
  // BLURRING
  //===========================================  

  ColorOcTree* blurred_treeB = new ColorOcTree(treeB->getResolution());
  convert_->createBlurredOctree(blurred_treeB);
  convert_->publishOctree(blurred_treeB, "OCTOMAP");
}

int main(int argc, char **argv)
{
  // Init Default
  ConvertOctomapConfig co_cfg;
  co_cfg.octomap_resolution = 0.1;
  co_cfg.blur_sigma = 0.1;
  AppConfig app_cfg;
  app_cfg.cloud1.clear();
  app_cfg.cloud2.clear();

  ConciseArgs parser(argc, argv, "");
  parser.add(app_cfg.cloud1, "a", "cloud1", "First point cloud.");
  parser.add(app_cfg.cloud2, "b", "cloud2", "Second point cloud.");
  parser.add(co_cfg.octomap_resolution, "r", "octomap_resolution", "Octree resolution.");
  parser.add(co_cfg.blur_sigma, "s", "blur_sigma", "Blurred octree sigma parameter.");
  parser.parse();

  //Set up LCM channel for visualization
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }

  App* app = new App(lcm, co_cfg, app_cfg);
  
  return 0;
}