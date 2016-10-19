// Start. Get 2 point clouds (either from user or default).
// Perform registration (input cloud aligned to reference).
// Convert to octomap, blur and save to file.
// Republish the octomap.
// Plot and store changes.

// Command: octrees-difference -h

#include <sstream>      // std::stringstream

#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

//#include <icp-registration/icp_3Dreg_and_plot.hpp>
//#include <icp-registration/clouds_io_utils.h>

#include "pointmatcher/PointMatcher.h"
#include <icp-registration/icp_utils.h>

#include <octrees-difference/convert_octomap.hpp>

//Compare
#include <octomap/octomap.h>
#include <octomap/math/Utils.h>

struct AppConfig
{
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

    string cloudA_name_;
    string cloudB_name_;

    bool do_convert_cloud_; // true if the pt cloud is ready to be converted to octomap
    bool do_republish_; // true if an octree is ready  

  private:
    
};

App::App(boost::shared_ptr< lcm::LCM >& lcm_, 
         ConvertOctomapConfig co_cfg_, AppConfig app_cfg_) : lcm_(lcm_), 
         co_cfg_(co_cfg_),
         app_cfg_(app_cfg_){
  convert_ = new ConvertOctomap(lcm_, co_cfg_); 
  std::cout << "Clouds to octrees conversion at launch.\n";

  cloudA_name_.clear();
  cloudB_name_.clear();

  do_convert_cloud_ = false;
  do_republish_ = false;
}

ColorOcTree* doConversion(App* app, DP &cloud, int cloud_idx); // NOTE: accepted cloud indexes are 0 (reference) and 1 (input).
//void convertCloudPclToPronto(pcl::PointCloud<pcl::PointXYZRGB> &cloud, pronto::PointCloud &cloud_out);
int validateArgs(const int argc, const char *argv[], ConvertOctomapConfig& co_cfg);
void usage(const char *argv[]);

int main(int argc, const char *argv[])
{
  // Init Default
  ConvertOctomapConfig co_cfg;
  co_cfg.octomap_resolution = 0.1; // was always 0.1 for mav and atlas
  co_cfg.blur_sigma = 0.1; // default was .5
  co_cfg.blur_map = false;
  AppConfig app_cfg;

  const int ret = validateArgs(argc, argv, co_cfg);

  if (ret == -1)
    return ret;

  //Set up LCM channel for visualization
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }

  App* app = new App(lcm, co_cfg, app_cfg);

  app->cloudA_name_.append(getenv("DRC_BASE"));
  app->cloudA_name_.append("/software/registration/data/cloud_01.vtk");
  app->cloudB_name_.append(getenv("DRC_BASE"));
  app->cloudB_name_.append("/software/registration/data/cloud_02.vtk");

  std::cout << app->cloudA_name_ << std::endl;
  std::cout << app->cloudB_name_ << std::endl;

  // Load point clouds from file
  DP cloudA = DP::load(app->cloudA_name_);
  DP cloudB = DP::load(app->cloudB_name_);

  //===========================================
  // CONVERT CLOUD TO OCTREE AND DIFFERENTIATE
  //=========================================== 

  app->do_convert_cloud_ = true; 

  if ( app->do_convert_cloud_ ) {

    ColorOcTree* treeA = doConversion(app, cloudA, 0);
    cout << "Pruned tree A size: " << treeA->size() <<" nodes." << endl;
    cout << "Changes A:" << endl;
    app->convert_->printChangesAndActual(*treeA);

    // Uncomment for visualization of matching results one by one:
    cout << "Press ENTER to continue..." << endl;
    cin.get();
    
    ColorOcTree* treeB = doConversion(app, cloudB, 1);
    cout << "Pruned tree B size: " << treeB->size() <<" nodes." << endl;
    cout << "Changes B:" << endl;
    app->convert_->printChangesByColor(*treeB);
  }
  
  return 0;
}

ColorOcTree* doConversion(App* app, DP &cloud, int cloud_idx)
{
  pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
  fromDataPointsToPCL(cloud, pcl_cloud);

  cout << "====================================================" << endl;
  cout << "Processing cloud with " << pcl_cloud.points.size()
       << " points" << endl;
  string current_cloud;  
  if (cloud_idx == 0)    
    current_cloud = "OCTOMAP_REF";
  else
    current_cloud = "OCTOMAP_IN";

  app->convert_->doWork(pcl_cloud, current_cloud); 

  cout << "Processing completed!" << endl;
  cout << "====================================================" << endl;
           
  app->do_republish_ = true;
  app->do_convert_cloud_ = false;

  if (app->do_republish_){
    //std::cout << "Republishing unblurred octomap.\n";
    app->convert_->publishOctree(app->convert_->getTree(),current_cloud);
  }

  return app->convert_->getTree();
}

// Make sure that the command arguments make sense
int validateArgs(const int argc, const char *argv[], ConvertOctomapConfig& co_cfg)
{
  const int endOpt(argc);

  for (int i = 1; i < endOpt; i += 2)
  {
    const string opt(argv[i]);
    if (i + 1 > endOpt)
    {
      cerr << "Incorrect use of option " << opt << ", usage:"; usage(argv); exit(1);
    }
    if (opt == "-r" || opt == "--octomapRes") {
      co_cfg.octomap_resolution = atof(argv[i+1]);
    }
    else if (opt == "-b" || opt == "--blurSigma") {
      co_cfg.blur_sigma = atof(argv[i+1]);
    }
    else if (opt == "-h")
    {
      cerr << "Usage:";
      usage(argv);
      return -1;
    }
    else
    {
      cerr << "Unknown option " << opt << ", usage:"; usage(argv); exit(1);
    }
  }
  return 0;
}

// Dump command-line help
void usage(const char *argv[])
{
  cerr << endl;
  cerr << "-r or --octomapRes --> Resolution of underlying octomap (default: 0.1)" << endl;
  cerr << "-b or --blurSigma --> Radius of the blur kernel (default: 0.1)" << endl;
  cerr << endl;
}