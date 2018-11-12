  /*===================================
  =                AICP               =
  ===================================*/
// Auto-tuned Iterative Closest Point (AICP) is a module for non-incremental point cloud registration
// and localization failure prediction. The registration strategy is based on the libpointmatcher framework (Pomerleau et al., AR 2012).

// AICP has been tested on Carnegie Robotics Multisense SL data from the NASA Valkyrie and Boston Dynamics Atlas humanoid robots,
// as well as the IIT HyQ quadruped and the Clearpath Husky mobile platform. The framework supports Lightweight Communications
// and Marshalling (LCM) integration for real-time message transfering.

// ===============
// Example Usage:
// ===============
// Help: aicp-registration-online -h
// Run: aicp-registration-online -s debug -b 83 -a -v

// Input: MULTISENSE_SCAN, POSE_BODY
// Output: POSE_BODY_CORRECTED
// Computes T_AICP (Nobili et al., ICRA 2017) and corrects the state estimate in POSE_BODY message.

// =========
// Details:
// =========
// The algorithm:
// 1. accumulates planar laser scans on a thread and generates 3D point clouds with -b scans
// 2. stores the first cloud as the reference cloud
// 3. before alignment, overlap and alignability parameters --> risk of alignment (Nobili et al., ICRA 2018, submitted) are computed
// 4. the reference cloud gets updated with latest accumulated cloud if (risk of alignment > threshold)
// 5. aligns each new point cloud to the current reference cloud
// 6. publishes a corrected body pose

#include <sstream>  // stringstream
#include <map>
#include <random>

#include "aicp_common_utils/cloudIO.h"

// yaml
#include "yaml-cpp/yaml.h" // read the yaml config

// lcm
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core/planar_lidar_t.hpp>
#include <lcmtypes/bot_core/rigid_transform_t.hpp>
#include <lcmtypes/bot_core/double_array_t.hpp>

// thread
// args
#include <ConciseArgs>

// tf
#include <bot_frames/bot_frames.h>

// pcl
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include "registration_apps/app_lcm.hpp"
#include "aicp_common_utils/common.hpp"

using namespace std;
using namespace aicp;
int main(int argc, char **argv){
  CommandLineConfig cl_cfg;
  cl_cfg.configFile.append(CONFIG_LOC);
  cl_cfg.configFile.append(PATH_SEPARATOR);
  cl_cfg.configFile.append("aicp_config.yaml");
  cl_cfg.working_mode = "robot";
  cl_cfg.failure_prediction_mode = 0;
  cl_cfg.verbose = FALSE;
  cl_cfg.apply_correction = FALSE;
  cl_cfg.pose_body_channel = "POSE_BODY";
  cl_cfg.output_channel = "POSE_BODY_CORRECTED"; // Create new channel...

  CloudAccumulateConfig ca_cfg;
  ca_cfg.batch_size = 240; // 240 is about 1 sweep
  ca_cfg.min_range = 0.50; //1.85; // remove all the short range points
  ca_cfg.max_range = 15.0; // we can set up to 30 meters (guaranteed range)
  ca_cfg.lidar_channel ="MULTISENSE_SCAN";
  //ca_cfg.check_local_to_scan_valid = FALSE;

  ConciseArgs parser(argc, argv, "aicp-registration-online");
  parser.add(cl_cfg.configFile, "c", "config_file", "Config file location");
  parser.add(cl_cfg.working_mode, "s", "working_mode", "Robot or Debug? (i.e. robot or debug)"); //Debug if I want to visualize moving frames in Director
  parser.add(cl_cfg.failure_prediction_mode, "u", "failure_prediction_mode", "Use: Alignment Risk (0), Degeneracy (1), ICN (2)");
  parser.add(cl_cfg.verbose, "v", "verbose", "Publish frames and clouds to LCM for debug");
  parser.add(cl_cfg.apply_correction, "a", "apply_correction", "Initialize ICP with corrected pose? (during debug)");
  parser.add(cl_cfg.pose_body_channel, "pc", "pose_body_channel", "Kinematics-inertia pose estimate");
  parser.add(cl_cfg.output_channel, "oc", "output_channel", "Corrected pose");
  parser.add(ca_cfg.lidar_channel, "l", "lidar_channel", "Input message e.g MULTISENSE_SCAN");
  parser.add(ca_cfg.batch_size, "b", "batch_size", "Number of scans per full 3D point cloud (at 5RPM)");
  parser.add(ca_cfg.min_range, "m", "min_range", "Closest accepted lidar range");
  parser.add(ca_cfg.max_range, "M", "max_range", "Furthest accepted lidar range");
  parser.parse();

  RegistrationParams registration_params;
  OverlapParams overlap_params;
  ClassificationParams classification_params;
  string experiments_param;
  /*===================================
  =            YAML Config            =
  ===================================*/
  string yamlConfig_;
  YAML::Node yn_;
  yamlConfig_ = cl_cfg.configFile;
  yn_ = YAML::LoadFile(yamlConfig_);

  YAML::Node registrationNode = yn_["AICP"]["Registration"];
  for(YAML::const_iterator it=registrationNode.begin();it != registrationNode.end();++it) {

    const string key = it->first.as<string>();

    if(key.compare("type") == 0) {
      registration_params.type = it->second.as<string>();
    }
    else if(key.compare("sensorRange") == 0) {
      registration_params.sensorRange =  it->second.as<float>();
    }
    else if(key.compare("sensorAngularView") == 0) {
      registration_params.sensorAngularView =  it->second.as<float>();
    }
    else if(key.compare("loadPosesFrom") == 0) {
      registration_params.loadPosesFrom = it->second.as<string>();
    }
    else if(key.compare("initialTransform") == 0) {
      registration_params.initialTransform = it->second.as<string>();
    }
    else if(key.compare("saveCorrectedPose") == 0) {
      registration_params.saveCorrectedPose =  it->second.as<bool>();
    }
    else if(key.compare("saveInitializedReadingCloud") == 0) {
      registration_params.saveInitializedReadingCloud =  it->second.as<bool>();
    }
    else if(key.compare("saveRegisteredReadingCloud") == 0) {
      registration_params.saveRegisteredReadingCloud =  it->second.as<bool>();
    }
    else if(key.compare("enableLcmVisualization") == 0) {
      registration_params.enableLcmVisualization =  it->second.as<bool>();
    }
  }
  if(registration_params.type.compare("Pointmatcher") == 0) {

    YAML::Node pointmatcherNode = registrationNode["Pointmatcher"];

    for(YAML::const_iterator it=pointmatcherNode.begin();it != pointmatcherNode.end();++it) {
      const string key = it->first.as<string>();

      if(key.compare("configFileName") == 0) {
        registration_params.pointmatcher.configFileName.append(FILTERS_CONFIG_LOC);
        registration_params.pointmatcher.configFileName.append(PATH_SEPARATOR);
        registration_params.pointmatcher.configFileName = FILTERS_CONFIG_LOC + PATH_SEPARATOR + it->second.as<string>();
      }
      else if(key.compare("printOutputStatistics") == 0) {
        registration_params.pointmatcher.printOutputStatistics =  it->second.as<bool>();
      }
    }
  }
  else if(registration_params.type.compare("GICP") == 0) {

    YAML::Node gicpNode = registrationNode["GICP"];

    for(YAML::const_iterator it=gicpNode.begin();it != gicpNode.end();++it) {
      const string key = it->first.as<string>();
      const float val = it->second.as<float>();
    }
  }
  YAML::Node overlapNode = yn_["AICP"]["Overlap"];
  for(YAML::const_iterator it=overlapNode.begin();it != overlapNode.end();++it) {

    const string key = it->first.as<string>();

    if(key.compare("type") == 0) {
      overlap_params.type = it->second.as<string>();
    }
  }
  if(overlap_params.type.compare("OctreeBased") == 0) {

    YAML::Node octreeBasedNode = overlapNode["OctreeBased"];

    for(YAML::const_iterator it=octreeBasedNode.begin();it != octreeBasedNode.end();++it) {
      const string key = it->first.as<string>();

      if(key.compare("octomapResolution") == 0) {
        overlap_params.octree_based.octomapResolution = it->second.as<float>();
      }
    }
  }
  YAML::Node classificationNode = yn_["AICP"]["Classifier"];
  for (YAML::const_iterator it = classificationNode.begin(); it != classificationNode.end(); ++it) {
    const std::string key = it->first.as<std::string>();

    if (key.compare("type") == 0) {
      classification_params.type = it->second.as<std::string>();
    }
  }

  if (classification_params.type.compare("SVM") == 0) {

    YAML::Node svmNode = classificationNode["SVM"];

    for(YAML::const_iterator it=svmNode.begin();it != svmNode.end();++it) {
      const std::string key = it->first.as<std::string>();

      if(key.compare("threshold") == 0) {
        classification_params.svm.threshold = it->second.as<double>();
      }
      else if(key.compare("trainingFile") == 0) {
        classification_params.svm.trainingFile = expandEnvironmentVariables(it->second.as<std::string>());
      }
      else if(key.compare("testingFile") == 0) {
          classification_params.svm.testingFile = expandEnvironmentVariables(it->second.as<std::string>());
      }
      else if(key.compare("saveFile") == 0) {
        classification_params.svm.saveFile = expandEnvironmentVariables(it->second.as<std::string>());
      }
      else if(key.compare("saveProbs") == 0) {
        classification_params.svm.saveProbs = expandEnvironmentVariables(it->second.as<std::string>());
      }
      else if(key.compare("modelLocation") == 0) {
        classification_params.svm.modelLocation = expandEnvironmentVariables(it->second.as<std::string>());
      }
    }
  }
  YAML::Node experimentsNode = yn_["AICP"]["Experiments"];
  for(YAML::const_iterator it=experimentsNode.begin();it != experimentsNode.end();++it) {

    const string key = it->first.as<string>();

    if(key.compare("type") == 0) {
      experiments_param = it->second.as<string>();
    }
  }

  cout << "============================" << endl
       << "Parsed YAML Config" << endl
       << "============================" << endl;

  cout << "[Main] Registration Type: "                 << registration_params.type                          << endl;
  cout << "[Main] Sensor Range: "                      << registration_params.sensorRange                   << endl;
  cout << "[Main] Sensor Angular View: "               << registration_params.sensorAngularView             << endl;
  cout << "[Main] Load Poses from: "                   << registration_params.loadPosesFrom                 << endl;
  cout << "[Main] Initial Transform: "                 << registration_params.initialTransform              << endl;
  cout << "[Main] Save Corrected Pose: "               << registration_params.saveCorrectedPose             << endl;
  cout << "[Main] Save Initialized Reading Cloud: "    << registration_params.saveInitializedReadingCloud   << endl;
  cout << "[Main] Save Registered Reading Cloud: "     << registration_params.saveRegisteredReadingCloud    << endl;
  cout << "[Main] Enable Lcm Visualization: "          << registration_params.enableLcmVisualization        << endl;

  if(registration_params.type.compare("Pointmatcher") == 0) {
    cout << "[Pointmatcher] Config File Name: "                << registration_params.pointmatcher.configFileName        << endl;
    cout << "[Pointmatcher] Print Registration Statistics: "   << registration_params.pointmatcher.printOutputStatistics << endl;
  }
  else if(registration_params.type.compare("GICP") == 0) {
  }

  cout << "[Main] Overlap Type: "                   << overlap_params.type                             << endl;

  if(overlap_params.type.compare("OctreeBased") == 0) {
    cout << "[OctreeBased] Octomap Resolution: "    << overlap_params.octree_based.octomapResolution   << endl;
  }

  cout << "[Main] Classification Type: "       << classification_params.type                    << endl;

  if(classification_params.type.compare("SVM") == 0) {
    cout << "[SVM] Acceptance Threshold: "    << classification_params.svm.threshold           << endl;
    cout << "[SVM] Training File: "           << classification_params.svm.trainingFile        << endl;
    cout << "[SVM] Testing File: "            << classification_params.svm.testingFile         << endl;
    cout << "[SVM] Saving Model To: "         << classification_params.svm.saveFile            << endl;
    cout << "[SVM] Saving Probs To: "         << classification_params.svm.saveProbs           << endl;
    cout << "[SVM] Loading Model From: "      << classification_params.svm.modelLocation       << endl;
  }

  cout << "[Main] Experiments Type: "               << experiments_param                               << endl;
  cout << "============================" << endl;


  // I'd rather use std::shared_ptr but cloud accumulator prevents it
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  AppLCM* app= new AppLCM(lcm, cl_cfg, ca_cfg,
                    registration_params, overlap_params,
                    classification_params, experiments_param);

  while(0 == lcm->handle());
  delete app;
}
