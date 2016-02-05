// drc-icp-plot-advanced

// Program for 3D clouds alignment
// using libpointmatcher (library for ICP).
//
// Options: - take 2 clouds from user (or default) and give 
//            transformation between them (-a reference, -b input)
//          - the alignment is performed in two consecutive phases:
//               1) ICP alignment using trimmed distance outlier filter
//                  (Hard rejection threshold using quantile. This filter considers 
//                   as inlier a certain percentage of the links with the smallest norms.)
//               2) ICP alignment using minimum distance outlier filter
//                  (This filter considers as outlier links whose norms are below a threshold.)
// Config files automatically set:
//               1) icp_3D_cfg_trimmed.yaml
//               2) icp_3D_cfg_max.yaml

#include <sstream>  

#include "icp_3Dreg_and_plot.hpp"

#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

#include "clouds_io_utils.h"

struct AppConfig
{
};

class App{
  public:
    App(boost::shared_ptr<lcm::LCM> &lcm_, 
        RegistrationConfig reg_cfg_, AppConfig app_cfg);
    
    ~App(){
    }  

    RegistrationConfig reg_cfg_;
    AppConfig app_cfg_;
    
    boost::shared_ptr<lcm::LCM> lcm_;
    Registration* registr_;  

  private:
    
};

App::App(boost::shared_ptr< lcm::LCM >& lcm_, 
         RegistrationConfig reg_cfg_, AppConfig app_cfg_) : lcm_(lcm_), 
         reg_cfg_(reg_cfg_),
         app_cfg_(app_cfg_){

  registr_ = new Registration(lcm_, reg_cfg_);
  std::cout << "Clouds matching at launch.\n";  
}

int validateArgs(const int argc, const char *argv[], RegistrationConfig& reg_cfg);
void usage(const char *argv[]);

int main(int argc, const char *argv[])
{
  // Init Default
  RegistrationConfig reg_cfg;
  reg_cfg.configFile3D_.clear();
  reg_cfg.initTrans_.clear();
  reg_cfg.initTrans_.append("0,0,0");
  AppConfig app_cfg;

  const int ret = validateArgs(argc, argv, reg_cfg);
  if (ret == -1)
    return ret;

  // Load first config file (trimmed distance outlier filter)
  reg_cfg.configFile3D_.append(reg_cfg.homedir);
  reg_cfg.configFile3D_.append("/oh-distro/software/perception/registration/filters_config/icp_3D_cfg_trimmed.yaml");

  if (reg_cfg.cloud_name_A.empty())
  {
    std::cerr <<"ERROR: reference point cloud missing." <<std::endl;	
  }
  else if (reg_cfg.cloud_name_B.empty())
  {    
  	std::cerr <<"ERROR: reading point cloud missing." <<std::endl;	
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
  // First ICP loop
  app->registr_->getICPTransform(data, ref);
  PM::TransformationParameters T1 = app->registr_->getTransform();
  cout << "3D Transformation (Trimmed Outlier Filter):" << endl << T1 << endl;

  // Second ICP loop
  DP out1 = app->registr_->getDataOut();
  string configName2;
  configName2.append(reg_cfg.homedir);
  configName2.append("/oh-distro/software/perception/registration/filters_config/icp_3D_cfg_max.yaml");
  app->registr_->setConfigFile(configName2);

  app->registr_->getICPTransform(out1, ref);
  PM::TransformationParameters T2 = app->registr_->getTransform();
  cout << "3D Transformation (Max Distance Outlier Filter):" << endl << T2 << endl;

  //=================================
  // ERROR
  //=================================
  DP out2 = app->registr_->getDataOut();
  float hausDist = hausdorffDistance(ref, out2);
  
  cout << "Hausdorff distance: " << hausDist << " m" << endl;

  PM::ICP icp = app->registr_->getIcp();
  float meanDist = pairedPointsMeanDistance(ref, out2, icp);
  
  cout << "Paired points mean distance: " << meanDist << " m" << endl;
  
  return 0;
}

// Make sure that the command arguments make sense
int validateArgs(const int argc, const char *argv[], RegistrationConfig& reg_cfg)
{
  const int endOpt(argc);

  for (int i = 1; i < endOpt; i += 2)
  {
    const string opt(argv[i]);
    if (i + 1 > endOpt)
    {
      cerr << "Incorrect use of option " << opt << ", usage:"; usage(argv); exit(1);
    }
    if (opt == "-i" || opt == "--initT") {
      reg_cfg.initTrans_.clear();
      reg_cfg.initTrans_ = argv[i+1];
    }
    else if (opt == "-a" || opt == "--reference") {
      reg_cfg.cloud_name_A.append(argv[i+1]);
    }
    else if (opt == "-b" || opt == "--input") {
      reg_cfg.cloud_name_B.append(argv[i+1]);
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
  //TODO: add new options --isTransfoSaved, --initTranslation, --initRotation
  cerr << endl << endl;
  cerr << "* To run ICP registration:" << endl;
  cerr << "  " << argv[0] << " [OPTIONS]" << endl;
  cerr << endl;
  cerr << "OPTIONS can be a combination of:" << endl;
  cerr << "-i or --initT --> [x,y,theta]  Initial transformation applyed to input cloud (default: 0,0,0)" << endl;
  cerr << "-a or --reference --> Reference cloud name  Load the .vtk file from current folder (default: multisense_00.vtk)" << endl;
  cerr << "-b or --input --> Reference cloud name  Load the .vtk file from current folder (default: multisense_01.vtk)" << endl;
  cerr << endl;
}