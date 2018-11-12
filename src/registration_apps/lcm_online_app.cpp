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
#include "registration_apps/yaml_configurator.hpp"

using namespace std;

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


  /*===================================
  =            YAML Config            =
  ===================================*/

  aicp::YAMLConfigurator yaml_conf;
  if(!yaml_conf.parse(cl_cfg.configFile)){
      cerr << "ERROR: could not parse file " << cl_cfg.configFile << endl;
      return -1;
  }
  // I'd rather use std::shared_ptr but cloud accumulator prevents it
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  aicp::AppLCM* app= new aicp::AppLCM(lcm,
                          cl_cfg,
                          ca_cfg,
                          yaml_conf.getRegistrationParams(),
                          yaml_conf.getOverlapParams(),
                          yaml_conf.getClassificationParams(),
                          yaml_conf.getExperimentParams());

  while(0 == lcm->handle());
  delete app;
  return 0;
}
