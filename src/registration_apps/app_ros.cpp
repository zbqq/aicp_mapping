#include "registration_apps/app_ros.hpp"

#include "aicp_registration/registration.hpp"
#include "aicp_overlap/overlap.hpp"
#include "aicp_classification/classification.hpp"

#include <tf_conversions/tf_eigen.h>

namespace aicp {

AppROS::AppROS(ros::NodeHandle &nh,
               const CommandLineConfig &cl_cfg,
               const ScanAccumulatorConfig &sa_cfg,
               const RegistrationParams &reg_params,
               const OverlapParams &overlap_params,
               const ClassificationParams &class_params,
               const string &bot_param_path) :
    App(cl_cfg, reg_params, overlap_params, class_params),
    nh_(nh), accu_config_(sa_cfg),
    accu_(nh, accu_config_)
{
    paramInit();

    // Data structure
    aligned_clouds_graph_ = new AlignedCloudsGraph();
    // Visualizer
    vis_ = new ROSVisualizer();

    cout << "============================" << endl
         << "Start..." << endl
         << "============================" << endl;

    // Create debug data folder
    data_directory_path_ << "/tmp/aicp_data";
    const char* path = data_directory_path_.str().c_str();
    boost::filesystem::path dir(path);
    if(boost::filesystem::exists(path))
        boost::filesystem::remove_all(path);
    if(boost::filesystem::create_directory(dir))
        cerr << "Create AICP debug data directory: " << path << endl;

    // Pose publisher
    if (cl_cfg_.working_mode == "debug")
    {
        pose_debug_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(cl_cfg_.output_channel,10);
    }

    // Instantiate objects
    registr_ = create_registrator(reg_params_);
    overlapper_ = create_overlapper(overlap_params_);
    classifier_ = create_classifier(class_params_);
}

void AppROS::robotPoseCallBack(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose_msg)
{
    Eigen::Isometry3d world_to_body;
    std::unique_lock<std::mutex> lock(robot_state_mutex_);
    {
        // Latest world -> body (pose prior)
        getPoseAsIsometry3d(pose_msg, world_to_body_msg_);
        //      world_to_body_msg_utime_ = pose_msg->header.stamp.toNSec() / 1000;
        world_to_body = world_to_body_msg_;
    }

    // Compute and publish correction, same frequency as input pose (if "debug" mode)
    if (cl_cfg_.working_mode == "debug")
    {
        geometry_msgs::PoseStamped msg_out;

        // Apply correction if available (identity otherwise)
        // TODO: this could be wrong and must be fixed to match cl_cfg_.working_mode == "robot" case
        corrected_pose_ = total_correction_ * world_to_body; // world -> reference =
                                                             // body -> reference * world -> body

        // Publish CORRECTED POSE
        tf::poseEigenToTF(corrected_pose_, temp_tf_pose_);
        tf::poseTFToMsg(temp_tf_pose_, msg_out.pose);
        msg_out.header.stamp = pose_msg->header.stamp;
        pose_debug_pub_.publish(msg_out);

        if (updated_correction_)
        {
            {
                std::unique_lock<std::mutex> lock(cloud_accumulate_mutex_);
                clear_clouds_buffer_ = TRUE;
            }
            updated_correction_ = FALSE;
        }
    }

    // TODO convert these messages to ROS
    /*
    // Publish current overlap
    bot_core::double_array_t msg_overlap;
    msg_overlap.utime = msg->utime;
    msg_overlap.num_values = 1;
    msg_overlap.values.push_back(octree_overlap_);
    lcm_->publish("OVERLAP",&msg_overlap);

    // Publish current alignability
    bot_core::double_array_t msg_al;
    msg_al.utime = msg->utime;
    msg_al.num_values = 1;
    msg_al.values.push_back(alignability_);
    lcm_->publish("ALIGNABILITY",&msg_al);

    if (!risk_prediction_.isZero() && !other_predictions_.empty())
    {
      // Publish current alignment risk
      bot_core::double_array_t msg_risk;
      msg_risk.utime = msg->utime;
      msg_risk.num_values = 1;
      msg_risk.values.push_back(risk_prediction_(0,0));
      lcm_->publish("ALIGNMENT_RISK",&msg_risk);

      // Publish current degeneracy
      bot_core::double_array_t msg_degen;
      msg_degen.utime = msg->utime;
      msg_degen.num_values = 1;
      msg_degen.values.push_back(other_predictions_.at(0));
      lcm_->publish("DEGENERACY",&msg_degen);
    }
*/
//    if ( !pose_initialized_ ){
//        world_to_body_corr_first_ = corrected_pose_;
//        world_to_body_msg_utime_first_ = pose_msg->header.stamp.toNSec() / 1000;
//    }

    pose_initialized_ = TRUE;
}

void AppROS::lidarScanCallBack(const sensor_msgs::LaserScan::ConstPtr &lidar_msg){
    if (!pose_initialized_){
        cout << "[App LCM] Pose estimate not initialized, waiting for pose prior...\n";
        return;
    }

    // Latest world -> body (pose prior)
    Eigen::Isometry3d world_to_body;
    {
        std::unique_lock<std::mutex> lock(robot_state_mutex_);
        world_to_body = world_to_body_msg_;
    }

//    // TODO retrieve these from somewhere (TF?)
//    body_to_lidar_ = Eigen::Isometry3d::Identity();
//    head_to_lidar_ = Eigen::Isometry3d::Identity();

//    Eigen::Isometry3d world_to_lidar_now = world_to_body_last * body_to_lidar_;
//    world_to_head_now_ = world_to_lidar_now * head_to_lidar_.inverse();

//    // 4. Store in LidarScan current scan wrt lidar frame
//    LidarScan* current_scan = new LidarScan(lidar_msg->header.stamp.toNSec() / 1000,
//                                            lidar_msg->angle_min,
//                                            lidar_msg->angle_increment,
//                                            lidar_msg->ranges,
//                                            lidar_msg->intensities,
//                                            world_to_head_now_,
//                                            head_to_lidar_,
//                                            world_to_body_last);

//    // Accumulate current scan in point cloud (projection to default reference "body")
//    if (!clear_clouds_buffer_)
//        // Accumulate EITHER always OR only when transition from walking to standing happens (DEPRECATED)
//        // Pyhton script convert_robot_behavior_type.py must be running.
//    {
//        if ( accu_.getCounter() % accu_config_.batch_size == 0 ) {
//            cout << "[Main] " << accu_.getCounter() << " of " << accu_config_.batch_size << " scans collected." << endl;
//        }
//        accu_.processLidar(lidar_msg);
//        lidar_scans_list_.push_back(*current_scan);
//    }
//    else
//    {
//        {
//            std::unique_lock<std::mutex> lock(cloud_accumulate_mutex_);
//            clear_clouds_buffer_ = FALSE;
//            cout << "[Main] Cleaning cloud buffer of " << accu_.getCounter() << " scans." << endl;
//        }

//        if ( accu_.getCounter() > 0 )
//            accu_.clearCloud();
//        if ( !lidar_scans_list_.empty() )
//            lidar_scans_list_.clear();
//    }

//    delete current_scan;

//    if ( accu_.getFinished() ){ //finished accumulating?
//        std::cout << "[Main] Finished Collecting: " << accu_.getFinishedTime() << std::endl;

//        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
//        // This class pc_vis is used only to convert an LCM point cloud into
//        // a PCL point cloud
//        // TODO remove the dependency on the accumulator!!
//        *cloud = accu_.getCloud();
//        // cloud = accu_.getCloud();
//        cloud->width = cloud->points.size();
//        cloud->height = 1;
//        cout << "[Main] Processing cloud with " << cloud->points.size() << " points." << endl;

//        // Push this cloud onto the work queue (mutex safe)
//        const int max_queue_size = 100;
//        {
//            std::unique_lock<std::mutex> lock(data_mutex_);
//            pcl::PointCloud<pcl::PointXYZ>::Ptr data (new pcl::PointCloud<pcl::PointXYZ>(*cloud));
//            data_queue_.push_back(data);
//            scans_queue_.push_back(lidar_scans_list_);
//            if (data_queue_.size() > max_queue_size) {
//                cout << "[Main] WARNING: dropping " <<
//                        (data_queue_.size()-max_queue_size) << " scans" << endl;
//            }
//            while (data_queue_.size() > max_queue_size) {
//                data_queue_.pop_front();
//                scans_queue_.pop_front();
//            }
//            lidar_scans_list_.clear();
//            accu_.clearCloud();
//        }
//        // Send notification to operator which is waiting for this condition variable
//        worker_condition_.notify_one();
//    }

}

void AppROS::getPoseAsIsometry3d(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose_msg,
                                 Eigen::Isometry3d& eigen_pose)
{
    tf::poseMsgToTF(pose_msg->pose.pose, temp_tf_pose_);
    tf::transformTFToEigen(temp_tf_pose_, eigen_pose);
}





void AppROS::run(){
    worker_thread_ = std::thread(std::ref(*this)); // std::ref passes a pointer for you behind the scene


}

}
