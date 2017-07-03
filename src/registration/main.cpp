// aicp-registration

// Test 3D point clouds alignment
//
// Options: - take 2 clouds from user and give
//            transformation between them (-a reference, -b reading)

#include <sstream>  // std::stringstream

// registration
#include "aicpRegistration/registration.hpp"
#include "aicpRegistration/common.hpp"

#include "commonUtils/cloudIO.h"
#include "commonUtils/common.hpp"

// yaml
#include "yaml-cpp/yaml.h" // read the yaml config

// args
#include <ConciseArgs>

using namespace std;
using namespace aicp;

struct AppConfig
{
  string configFile;
  string pointCloudA;
  string pointCloudB;
};

class App{
  public:
    App(AppConfig app_cfg);
    
    ~App(){
    }  

    AppConfig app_cfg_;
    
    //Registration* registr_;

  private:
    
};

App::App(AppConfig app_cfg_) : app_cfg_(app_cfg_){

  //registr_ = new Registration(lcm_, reg_cfg_);
  //std::cout << "Clouds matching at launch.\n";
}

int main(int argc, char **argv)
{
  /*RegistrationConfig reg_cfg;
  reg_cfg.configFile3D_.clear();
  reg_cfg.initTrans_.clear();
  reg_cfg.initTrans_.append("0,0,0");
*/
  AppConfig app_cfg;
  app_cfg.configFile.append(CONFIG_LOC);
  app_cfg.configFile.append(PATH_SEPARATOR);
  app_cfg.configFile.append("aicp_config.yaml");
  app_cfg.pointCloudA = "";
  app_cfg.pointCloudB = "";

  ConciseArgs parser(argc, argv, "test-registration");
  parser.add(app_cfg.configFile, "c", "config_file", "Config file location");
  parser.add(app_cfg.pointCloudA, "a", "point_cloud_reference", "Pointcloud A");
  parser.add(app_cfg.pointCloudB, "a", "point_cloud_reading", "Pointcloud B");
  parser.parse();
/*
  if (reg_cfg.cloud_name_A.empty())
  {
    reg_cfg.cloud_name_A.append(reg_cfg.homedir);
    reg_cfg.cloud_name_A.append("/logs/multisenselog__2015-11-16/pointclouds/multisense_00.vtk");	
  }
  if (reg_cfg.cloud_name_B.empty())
  {    
  	reg_cfg.cloud_name_B.append(reg_cfg.homedir);
    reg_cfg.cloud_name_B.append("/logs/multisenselog__2015-11-16/pointclouds/multisense_01.vtk");	
  }

  //Set up LCM channel for visualization
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }

  App* app = new App(lcm, reg_cfg, app_cfg);     
  
  // Load point clouds from file
  DP ref = DP::load(reg_cfg.cloud_name_A);
  DP data = DP::load(reg_cfg.cloud_name_B);

  //=================================
  // TRANSFORM 3D CLOUD
  //=================================

  app->registr_->getICPTransform(data, ref);
  
  PM::TransformationParameters T = app->registr_->getTransform();
  cout << "3D Transformation:" << endl << T << endl;

  //=================================
  // ERROR
  //=================================
  DP out = app->registr_->getDataOut();
  float hausDist = hausdorffDistance(ref, out);
  
  cout << "Hausdorff distance: " << hausDist << " m" << endl;

  PM::ICP icp = app->registr_->getIcp();
  float meanDist = pairedPointsMeanDistance(ref, out, icp);
  
  cout << "Paired points mean distance: " << meanDist << " m" << endl;

  // Publish clouds: plot in pronto visualizer
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr ref_plot = app->registr_->getCloud(0);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_plot = app->registr_->getCloud(2);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr result_plot = app->registr_->getCloud(1);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr init_in_plot = app->registr_->getCloud(3);

  app->registr_->publishCloud(app->pc_vis_, 60001, ref_plot);
  app->registr_->publishCloud(app->pc_vis_, 60002, in_plot);
  app->registr_->publishCloud(app->pc_vis_, 60003, result_plot);
  app->registr_->publishCloud(app->pc_vis_, 60007, init_in_plot);*/
  
  return 0;
}
