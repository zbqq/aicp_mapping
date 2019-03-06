  /*   __  ___  ____
 / _\ (  )/ __)(  _ \
/    \ )(( (__  ) __/
\_/\_/(__)\___)(_*/

// Auto-tuned Iterative Closest Point (AICP) is a module for point cloud registration (Nobili et al., ICRA 2017)
// and localization failure prediction (Nobili et al., ICRA 2018). The registration strategy is based on
// the libpointmatcher framework (Pomerleau et al., AR 2012).

// AICP has been tested on Carnegie Robotics Multisense SL data from the NASA Valkyrie and Boston Dynamics Atlas
// humanoid robots, as well as the IIT HyQ quadruped and the Clearpath Husky mobile platform.
// The framework supports Lightweight Communications and Marshalling (LCM) integration for real-time
// message transfering.

// ===============
// Example Usage:
// ===============
// Help: rosrun aicp aicp-lcm-online -h
// Run:  rosrun aicp aicp-lcm-online -s debug -b 80 -ar -f 5 -v

// Input (default): MULTISENSE_SCAN, POSE_BODY
// Output (default): POSE_BODY_CORRECTED
// Computes T_aicp (Nobili et al., ICRA 2017) and publishes POSE_BODY_CORRECTED usign T_aicp

// =========
// Details:
// =========
// The algorithm:
// 1. accumulates planar laser scans on a thread and generates 3D point clouds with -b scans
// 2. stores the first cloud as the reference cloud
// 3. before alignment, overlap and alignability parameters --> risk of alignment are computed
// 4. the reference cloud gets updated with latest accumulated cloud if (risk of alignment > threshold)
// 5. or with latest accumulated and aligned cloud every # clouds if windowed update is enabled (e.g -f 5)
// 6. aligns each new point cloud to the current reference cloud
// 7. publishes a corrected body pose

#include <sstream>  // stringstream
#include <map>
#include <random>

// lcm
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core/planar_lidar_t.hpp>
#include <lcmtypes/bot_core/rigid_transform_t.hpp>
#include <lcmtypes/bot_core/double_array_t.hpp>

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

#include "aicp_lcm/app_lcm.hpp"
#include "aicp_registration/yaml_configurator.hpp"
#include "aicp_utils/common.hpp"

using namespace std;

int main(int argc, char **argv){
    CommandLineConfig cl_cfg;
    cl_cfg.config_file.append(CONFIG_LOC);
    cl_cfg.config_file.append(PATH_SEPARATOR);
    cl_cfg.config_file.append("aicp_config.yaml");
    cl_cfg.working_mode = "robot"; // e.g. robot - POSE_BODY has been already corrected
                                   // or debug - apply previous transforms to POSE_BODY
    cl_cfg.failure_prediction_mode = FALSE; // compute Alignment Risk
    cl_cfg.reference_update_frequency = 5;

    cl_cfg.pose_body_channel = "POSE_BODY";
    cl_cfg.output_channel = "POSE_BODY_CORRECTED"; // Create new channel...
    cl_cfg.verbose = FALSE; // enable visualization for debug

    CloudAccumulateConfig ca_cfg;
    ca_cfg.batch_size = 80; // 240 is about 1 sweep at 5RPM // 80 is about 1 sweep at 15RPM
    ca_cfg.min_range = 0.50; // 1.85; // remove all the short range points
    ca_cfg.max_range = 15.0; // we can set up to 30 meters (guaranteed range)
    ca_cfg.lidar_channel ="MULTISENSE_SCAN";
    //ca_cfg.check_local_to_scan_valid = FALSE;

    ConciseArgs parser(argc, argv, "aicp-lcm-online");
    parser.add(cl_cfg.config_file, "c", "config_file", "AICP config file location");
    parser.add(cl_cfg.working_mode, "s", "working_mode", "Robot or debug?");
    parser.add(cl_cfg.failure_prediction_mode, "ar", "failure_prediction_mode", "Alignment Risk for reference update");
    parser.add(cl_cfg.reference_update_frequency, "f", "reference_update_frequency", "Reference update frequency (number of clouds)");

    parser.add(cl_cfg.pose_body_channel, "pc", "pose_body_channel", "Prior pose estimate");
    parser.add(cl_cfg.output_channel, "oc", "output_channel", "Corrected pose estimate");
    parser.add(cl_cfg.verbose, "v", "verbose", "Enable visualization to LCM for debug");

    parser.add(ca_cfg.batch_size, "b", "batch_size", "Number of planar scans per 3D point cloud");
    parser.add(ca_cfg.min_range, "m", "min_range", "Min accepted lidar range");
    parser.add(ca_cfg.max_range, "M", "max_range", "Max accepted lidar range");
    parser.add(ca_cfg.lidar_channel, "l", "lidar_channel", "Input message e.g MULTISENSE_SCAN");
    parser.parse();


    /*===================================
    =            YAML Config            =
    ===================================*/
    aicp::YAMLConfigurator yaml_conf;
    if(!yaml_conf.parse(cl_cfg.config_file)){
      cerr << "ERROR: could not parse AICP config file." << cl_cfg.config_file << endl;
      return -1;
    }
    yaml_conf.printParams();

    /*===================================
    =             Create LCM            =
    ===================================*/
    // I'd rather use std::shared_ptr but cloud accumulator prevents it
    boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
    if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good." <<std::endl;
    }

    /*===================================
    =              Start App            =
    ===================================*/
    aicp::AppLCM* app = new aicp::AppLCM(lcm,
                                         cl_cfg,
                                         ca_cfg,
                                         yaml_conf.getRegistrationParams(),
                                         yaml_conf.getOverlapParams(),
                                         yaml_conf.getClassificationParams());

    while(0 == lcm->handle());
    delete app;
    return 0;
}
