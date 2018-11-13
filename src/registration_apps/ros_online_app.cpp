#include "registration_apps/app_ros.hpp"
#include "registration_apps/yaml_configurator.hpp"
#include "aicp_common_utils/common.hpp"


int main(int argc, char** argv){
    ros::init(argc, argv, "aicp_registration_node");
    ros::NodeHandle nh("~");

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

    nh.getParam("config_file", cl_cfg.configFile);
    nh.getParam("working_mode", cl_cfg.working_mode);
    nh.getParam("failure_prediction_mode", cl_cfg.failure_prediction_mode);
    nh.getParam("verbose", cl_cfg.verbose);
    nh.getParam("apply_correction", cl_cfg.apply_correction);
    nh.getParam("pose_body_channel", cl_cfg.pose_body_channel);
    nh.getParam("output_channel", cl_cfg.output_channel);

    CloudAccumulateConfig ca_cfg;

    ca_cfg.batch_size = 240; // 240 is about 1 sweep
    ca_cfg.min_range = 0.50; //1.85; // remove all the short range points
    ca_cfg.max_range = 15.0; // we can set up to 30 meters (guaranteed range)
    ca_cfg.lidar_channel ="MULTISENSE_SCAN";

    nh.getParam("lidar_channel", ca_cfg.lidar_channel);
    nh.getParam("batch_size", ca_cfg.batch_size);
    nh.getParam("min_range", ca_cfg.min_range);
    nh.getParam("max_range", ca_cfg.max_range);

    std::string bot_param_path = "";
    nh.getParam("bot_param_path", bot_param_path);

    aicp::YAMLConfigurator yaml_conf;
    if(!yaml_conf.parse(cl_cfg.configFile)){
        cerr << "ERROR: could not parse file " << cl_cfg.configFile << endl;
        return -1;
    }

    std::shared_ptr<aicp::AppROS> my_app(new aicp::AppROS(nh,
                                                          cl_cfg,
                                                          ca_cfg,
                                                          yaml_conf.getRegistrationParams(),
                                                          yaml_conf.getOverlapParams(),
                                                          yaml_conf.getClassificationParams(),
                                                          yaml_conf.getExperimentParams(),
                                                          bot_param_path));

    ros::Subscriber lidar_sub = nh.subscribe(ca_cfg.lidar_channel, 100, &aicp::AppROS::lidarScanCallBack, my_app.get());
    ros::Subscriber pose_sub = nh.subscribe(cl_cfg.pose_body_channel, 100, &aicp::AppROS::robotPoseCallBack, my_app.get());

    my_app->run();
    ros::spin();

}
