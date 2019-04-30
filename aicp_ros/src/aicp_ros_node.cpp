#include "aicp_ros/app_ros.hpp"
#include "aicp_registration/yaml_configurator.hpp"
#include "aicp_utils/common.hpp"

using namespace std;

int main(int argc, char** argv){
    ros::init(argc, argv, "aicp_ros_node");
    ros::NodeHandle nh("~");

    CommandLineConfig cl_cfg;
    cl_cfg.registration_config_file.append("");
    cl_cfg.aicp_config_file.append("");
    cl_cfg.working_mode = "robot"; // e.g. robot - POSE_BODY has been already corrected
                                   // or debug - apply previous transforms to POSE_BODY
    cl_cfg.fixed_frame = "map";
    cl_cfg.load_map_from_file = false; // if enabled, wait for file_path to be sent through a service,
                                       // align first cloud only against map (to visualize final drift)
    cl_cfg.map_from_file_path = "";
    cl_cfg.localize_against_prior_map = false; // reference is prior map cropped around current pose
    cl_cfg.localize_against_built_map = false; // reference is aligned map cropped around current pose
    cl_cfg.crop_map_around_base = 8.0; // rectangular box dimesions: value*2 x value*2
    cl_cfg.merge_aligned_clouds_to_map = false; // improves performance if trajectory goes
                                                // outside map (issue: slow)

    cl_cfg.failure_prediction_mode = false; // compute Alignment Risk
    cl_cfg.reference_update_frequency = 5;
    cl_cfg.max_correction_magnitude = 0.5; // Max allowed correction magnitude
                                           // (probably failed alignment otherwise)
    cl_cfg.max_queue_size = 3; // maximum length of the queue of accumulated point clouds. was 100 previously

    cl_cfg.pose_body_channel = "/state_estimator/pose_in_odom";
    cl_cfg.output_channel = "/aicp/pose_corrected"; // Create new channel...
    cl_cfg.verbose = false; // enable visualization for debug
    cl_cfg.write_input_clouds_to_file = false; // write the raw incoming point clouds to a folder, for post processing
    cl_cfg.process_input_clouds_from_file = false;  // process raw incoming point cloud from a folder
    cl_cfg.process_input_clouds_folder = "/tmp/aicp_data";

    aicp::VelodyneAccumulatorConfig va_cfg;
    va_cfg.batch_size = 80; // 240 is about 1 sweep at 5RPM // 80 is about 1 sweep at 15RPM
    va_cfg.min_range = 0.50; // 1.85; // remove all the short range points
    va_cfg.max_range = 15.0; // we can set up to 30 meters (guaranteed range)
    va_cfg.lidar_topic ="/point_cloud_filter/velodyne/point_cloud_filtered";
    va_cfg.inertial_frame = "/odom";

    nh.getParam("registration_config_file", cl_cfg.registration_config_file);
    nh.getParam("aicp_config_file", cl_cfg.aicp_config_file);
    nh.getParam("working_mode", cl_cfg.working_mode);
    nh.getParam("fixed_frame", cl_cfg.fixed_frame);
    nh.getParam("load_map_from_file", cl_cfg.load_map_from_file);
    nh.getParam("map_from_file_path", cl_cfg.map_from_file_path);
    nh.getParam("localize_against_prior_map", cl_cfg.localize_against_prior_map);
    nh.getParam("localize_against_built_map", cl_cfg.localize_against_built_map);
    nh.getParam("crop_map_around_base", cl_cfg.crop_map_around_base);
    nh.getParam("merge_aligned_clouds_to_map", cl_cfg.merge_aligned_clouds_to_map);

    nh.getParam("failure_prediction_mode", cl_cfg.failure_prediction_mode);
    nh.getParam("reference_update_frequency", cl_cfg.reference_update_frequency);
    nh.getParam("max_correction_magnitude", cl_cfg.max_correction_magnitude);
    nh.getParam("max_queue_size", cl_cfg.max_queue_size);

    nh.getParam("pose_body_channel", cl_cfg.pose_body_channel);
    nh.getParam("output_channel", cl_cfg.output_channel);
    nh.getParam("verbose", cl_cfg.verbose);
    nh.getParam("write_input_clouds_to_file", cl_cfg.write_input_clouds_to_file);
    nh.getParam("process_input_clouds_from_file", cl_cfg.process_input_clouds_from_file);
    nh.getParam("process_input_clouds_folder", cl_cfg.process_input_clouds_folder);


    nh.getParam("batch_size", va_cfg.batch_size);
    nh.getParam("min_range", va_cfg.min_range);
    nh.getParam("max_range", va_cfg.max_range);
    nh.getParam("lidar_channel", va_cfg.lidar_topic);
    nh.getParam("inertial_frame", va_cfg.inertial_frame);



    /*===================================
    =            YAML Config            =
    ===================================*/
    aicp::YAMLConfigurator yaml_conf;
    if(!yaml_conf.parse(cl_cfg.aicp_config_file)){
        cerr << "ERROR: could not parse file " << cl_cfg.aicp_config_file << endl;
        return -1;
    }
    yaml_conf.printParams();

    RegistrationParams reg_params = yaml_conf.getRegistrationParams();
    if(!nh.getParam("registration_default_config_file", reg_params.pointmatcher.configFileName)){
        ROS_ERROR("Param \"registration_default_config_file not found!\"");
    }

    OverlapParams overlap_params = yaml_conf.getOverlapParams();

    ClassificationParams classification_params = yaml_conf.getClassificationParams();
    nh.getParam("trainingFile", classification_params.svm.trainingFile);
    nh.getParam("testingFile", classification_params.svm.testingFile);
    nh.getParam("saveFile", classification_params.svm.saveFile);
    nh.getParam("saveProbs", classification_params.svm.saveProbs);
    nh.getParam("modelLocation", classification_params.svm.modelLocation);

    /*===================================
    =              Start App            =
    ===================================*/
    std::shared_ptr<aicp::AppROS> app(new aicp::AppROS(nh,
                                                       cl_cfg,
                                                       va_cfg,
                                                       reg_params,
                                                       overlap_params,
                                                       classification_params));

    if (!cl_cfg.process_input_clouds_from_file){

        // Subscribers
        ros::Subscriber lidar_sub = nh.subscribe(va_cfg.lidar_topic, 100, &aicp::AppROS::velodyneCallBack, app.get());
        ros::Subscriber pose_sub = nh.subscribe(cl_cfg.pose_body_channel, 100, &aicp::AppROS::robotPoseCallBack, app.get());
        ros::Subscriber marker_sub = nh.subscribe("/interaction_marker/pose", 100, &aicp::AppROS::interactionMarkerCallBack, app.get());

        // Advertise services (using service published by anybotics icp_tools ui)
        ros::ServiceServer load_map_server_ = nh.advertiseService("/icp_tools/load_map_from_file", &aicp::AppROS::loadMapFromFileCallBack, app.get());
        ros::ServiceServer go_back_server_ = nh.advertiseService("/aicp/go_back_request", &aicp::AppROS::goBackRequestCallBack, app.get());

        ROS_INFO_STREAM("[Aicp] Waiting for input messages...");

        app->run();
        ros::spin();

    }else{
        app->processFromFile(cl_cfg.process_input_clouds_folder);
    }

    return 0;
}
