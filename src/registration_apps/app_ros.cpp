#include "registration_apps/app_ros.hpp"

#include "aicp_registration/registration.hpp"
#include "aicp_overlap/overlap.hpp"
#include "aicp_classification/classification.hpp"
#include "aicp_common_utils/common.hpp"

#include <tf_conversions/tf_eigen.h>

#include <pcl/io/ply_io.h>

namespace aicp {

AppROS::AppROS(ros::NodeHandle &nh,
               const CommandLineConfig &cl_cfg,
               const VelodyneAccumulatorConfig &va_cfg,
               const RegistrationParams &reg_params,
               const OverlapParams &overlap_params,
               const ClassificationParams &class_params,
               const string &bot_param_path) :
    App(cl_cfg, reg_params, overlap_params, class_params),
    nh_(nh), accu_config_(va_cfg)
{
    paramInit();

    // Data structure
    aligned_clouds_graph_ = new AlignedCloudsGraph();
    // Accumulator
    accu_ = new VelodyneAccumulatorROS(nh_, accu_config_);
    // Visualizer
    vis_ = new ROSVisualizer(nh_);

    // Init pose to identity
    world_to_body_ = Eigen::Isometry3d::Identity();
    world_to_body_previous_ = Eigen::Isometry3d::Identity();

    // Create debug data folder
    data_directory_path_ << "/tmp/aicp_data";
    const char* path = data_directory_path_.str().c_str();
    boost::filesystem::path dir(path);
    if(boost::filesystem::exists(path))
        boost::filesystem::remove_all(path);
    if(boost::filesystem::create_directory(dir))
    {
        cout << "Create AICP debug data directory: " << path << endl
             << "============================" << endl;
    }

    // Pose publisher
    if (cl_cfg_.working_mode == "debug")
    {
        corrected_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(cl_cfg_.output_channel,10);
    }
    // Verbose publishers
    if (cl_cfg_.verbose)
    {
        overlap_pub_ = nh_.advertise<std_msgs::Float32>("/aicp/overlap",10);
        alignability_pub_ = nh_.advertise<std_msgs::Float32>("/aicp/alignability",10);
        risk_pub_ = nh_.advertise<std_msgs::Float32>("/aicp/alignment_risk",10);
    }

    // Instantiate objects
    registr_ = create_registrator(reg_params_);
    overlapper_ = create_overlapper(overlap_params_);
    classifier_ = create_classifier(class_params_);
}

void AppROS::robotPoseCallBack(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose_msg_in)
{
    if ((cl_cfg_.load_map_from_file || cl_cfg_.localize_against_map)
         && !pose_marker_initialized_){
        ROS_WARN_STREAM("[Aicp] Pose initial guess in map not set, waiting for interactive marker...");
        return;
    }

    std::unique_lock<std::mutex> lock(robot_state_mutex_);
    {
        // Latest world -> body (pose prior)
        getPoseAsIsometry3d(pose_msg_in, world_to_body_msg_);
        world_to_body_ = world_to_body_msg_;
    }

    if (!pose_initialized_){
        world_to_body_previous_ = world_to_body_;

        // Initialize transform: pose_in_odom -> interactive_marker
        if (cl_cfg_.load_map_from_file || cl_cfg_.localize_against_map)
        {
            initialT_ = (world_to_body_marker_msg_ * world_to_body_.inverse()).matrix().cast<float>();
            total_correction_ = fromMatrix4fToIsometry3d(initialT_);
        } // identity otherwise
        ROS_INFO_STREAM("[Aicp] Starting localization...");
    }

    // TO_DO: remove modes -> deprecated in ROS as corrected trajectory not fed back to controller
    // Compute and publish correction, same frequency as input pose (if "debug" mode)
    if (cl_cfg_.working_mode == "debug")
    {
        geometry_msgs::PoseStamped pose_msg_out;

        // Apply correction if available (identity otherwise)
        // TODO: this could be wrong and must be fixed to match cl_cfg_.working_mode == "robot" case
        corrected_pose_ = total_correction_ * world_to_body_; // world -> reference =
                                                              // body -> reference * world -> body
        // Publish initial guess interactive marker
        if (!pose_initialized_)
            vis_->publishPose(corrected_pose_, 0, "", ros::Time::now().toNSec() / 1000);

        // Publish /aicp/pose_corrected
        tf::poseEigenToTF(corrected_pose_, temp_tf_pose_);
        tf::poseTFToMsg(temp_tf_pose_, pose_msg_out.pose);
        pose_msg_out.header = pose_msg_in->header;
        corrected_pose_pub_.publish(pose_msg_out);

        if ( updated_correction_ )
        {
            {
                std::unique_lock<std::mutex> lock(cloud_accumulate_mutex_);
                clear_clouds_buffer_ = TRUE;
            }
            updated_correction_ = FALSE;
        }
    }

    if (cl_cfg_.verbose)
    {
        // Publish /aicp/overlap
        std_msgs::Float32 overlap_msg;
        overlap_msg.data = octree_overlap_;
        overlap_pub_.publish(overlap_msg);

        // Publish /aicp/alignability
        std_msgs::Float32 alignability_msg;
        alignability_msg.data = alignability_;
        alignability_pub_.publish(alignability_msg);

        if (!risk_prediction_.isZero())
        {
            // Publish /aicp/alignment_risk
            std_msgs::Float32 risk_msg;
            risk_msg.data = risk_prediction_(0,0);
            risk_pub_.publish(risk_msg);
        }
    }

    pose_initialized_ = TRUE;
}

void AppROS::velodyneCallBack(const sensor_msgs::PointCloud2::ConstPtr &laser_msg_in){
    if (!pose_initialized_){
        ROS_WARN_STREAM("[Aicp] Pose not initialized, waiting for pose prior...");
        return;
    }

    // Accumulate planar scans to 3D point cloud (global frame)
    if (!clear_clouds_buffer_ )
    {
        accu_->processLidar(laser_msg_in);
//        cout << "[App ROS] " << accu_->getCounter() + 1 << " of " << accu_config_.batch_size << " scans collected." << endl;
    }
    else
    {
        {
            std::unique_lock<std::mutex> lock(cloud_accumulate_mutex_);
            clear_clouds_buffer_ = FALSE;
//            cout << "[App ROS] Cleaning cloud buffer of " << accu_->getCounter() << " scans." << endl;
        }

        if ( accu_->getCounter() > 0 )
            accu_->clearCloud();
    }

    // Ensure robot moves between accumulated clouds
    Eigen::Isometry3d relative_motion = world_to_body_previous_.inverse() * world_to_body_;
    double dist = relative_motion.translation().norm();
    double rpy[3];
    quat_to_euler(Eigen::Quaterniond(relative_motion.rotation()), rpy[0], rpy[1], rpy[2]);

    if ( accu_->getFinished() )//finished accumulating?
    {
        if ((dist > 1.0) ||
            fabs(rpy[0]) > (10.0 * M_PI / 180.0) || // condition on roll
            fabs(rpy[1]) > (10.0 * M_PI / 180.0) || // condition on pitch
            fabs(rpy[2]) > (10.0 * M_PI / 180.0))   // condition on yaw
        {
            std::cout << "[App ROS] Finished collecting time: " << accu_->getFinishedTime() << std::endl;

            pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
            *accumulated_cloud = accu_->getCloud();
            cout << "[App ROS] Processing cloud with " << accumulated_cloud->points.size() << " points." << endl;

//            vis_->publishCloud(accumulated_cloud, 10, "/aicp/accumulated_cloud", accu_->getFinishedTime());

            // Push this cloud onto the work queue (mutex safe)
            const int max_queue_size = 100;
            {
                std::unique_lock<std::mutex> lock(data_mutex_);

                // Populate AlignedCloud data structure
                AlignedCloudPtr current_cloud (new AlignedCloud(accu_->getFinishedTime(),
                                                                accumulated_cloud,
                                                                world_to_body_));
                world_to_body_previous_ = world_to_body_;

                // Stack current cloud into queue
                cloud_queue_.push_back(current_cloud);

                if (cloud_queue_.size() > max_queue_size) {
                    cout << "[App ROS] WARNING: dropping " <<
                            (cloud_queue_.size()-max_queue_size) << " clouds." << endl;
                }
                while (cloud_queue_.size() > max_queue_size) {
                    cloud_queue_.pop_front();
                }
            }
        }
        accu_->clearCloud();

        // Send notification to operator()() which is waiting for this condition variable
        worker_condition_.notify_one();
    }
}

void AppROS::interactionMarkerCallBack(const geometry_msgs::PoseStampedConstPtr& init_pose_msg_in)
{
    if (!cl_cfg_.load_map_from_file && !cl_cfg_.localize_against_map){
        ROS_WARN_STREAM("[Aicp] Map service disabled - interactive marker neglected.");
        return;
    }
    if (!map_initialized_){
        ROS_WARN_STREAM("[Aicp] Map not initialized, waiting for map service...");
        return;
    }
    if (!pose_initialized_){ // initial pose can be updated by user until localization starts, not after.
        ROS_INFO_STREAM("[Aicp] Set localization initial pose in map.");

        // world -> body initial guess from interactive marker
        getPoseAsIsometry3d(init_pose_msg_in, world_to_body_marker_msg_);

        pose_marker_initialized_ = TRUE;
    }
    else
        ROS_WARN_STREAM("[Aicp] Interactive marker cannot be updated after localization started!");
}

bool AppROS::loadMapFromFileCallBack(aicp::ProcessFile::Request& request, aicp::ProcessFile::Response& response)
{
    response.success = loadMapFromFile(request.file_path);
    return true;
}

bool AppROS::loadMapFromFile(const std::string& file_path)
{
    if ((!cl_cfg_.load_map_from_file && !cl_cfg_.localize_against_map)
         || map_initialized_){
        if(map_initialized_)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr map = map_->getCloud();
            vis_->publishMap(map, map_->getUtime());
        }
        ROS_WARN_STREAM("[Aicp] Map service disabled or map already loaded!");
        return false;
    }

    // Load map from file
    ROS_INFO_STREAM("[Aicp] Loading map from '" << file_path << "' ...");
    pcl::PointCloud<pcl::PointXYZ>::Ptr map (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPLYFile<pcl::PointXYZ>(file_path.c_str(), *map) == -1)
    {
      ROS_ERROR_STREAM("[Aicp] Error loading map from file!");
      return false;
    }
    ROS_INFO_STREAM("[Aicp] Loaded map with " << map->points.size() << " points.");

    // Pre-filter map
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_map (new pcl::PointCloud<pcl::PointXYZ>);
    regionGrowingUniformPlaneSegmentationFilter(map, filtered_map);
    // Populate map object
    map_ = new AlignedCloud(ros::Time::now().toNSec() / 1000,
                            filtered_map,
                            Eigen::Isometry3d::Identity());

    map_initialized_ = TRUE;
    vis_->publishMap(map, map_->getUtime());

    return true;
}

void AppROS::getPoseAsIsometry3d(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose_msg,
                                 Eigen::Isometry3d& eigen_pose)
{
    tf::poseMsgToTF(pose_msg->pose.pose, temp_tf_pose_);
    tf::transformTFToEigen(temp_tf_pose_, eigen_pose);
}

void AppROS::getPoseAsIsometry3d(const geometry_msgs::PoseStampedConstPtr &pose_msg,
                                 Eigen::Isometry3d& eigen_pose)
{
    tf::poseMsgToTF(pose_msg->pose, temp_tf_pose_);
    tf::transformTFToEigen(temp_tf_pose_, eigen_pose);
}

void AppROS::run()
{
    worker_thread_ = std::thread(std::ref(*this)); // std::ref passes a pointer for you behind the scene
}

} // namespace aicp
