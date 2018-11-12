#include "registration_apps/app_ros.hpp"
#include "aicp_registration/registration.hpp"
#include "aicp_overlap/overlap.hpp"
#include "aicp_classification/classification.hpp"

namespace aicp {

AppROS::AppROS(ros::NodeHandle &nh,
               const CommandLineConfig &cl_cfg,
               CloudAccumulateConfig ca_cfg,
               RegistrationParams reg_params,
               OverlapParams overlap_params,
               ClassificationParams class_params,
               string exp_params) :
    App(cl_cfg, reg_params, overlap_params, class_params, exp_params),
    nh_(nh)
{
    paramInit();


}

void AppROS::lidarScanCallBack(const sensor_msgs::LaserScan::ConstPtr &lidar_msg){
    if (!pose_initialized_){
      cout << "[Main] Pose estimate not initialized, waiting...\n";
      return;
    }

    // 1. copy robot state
    Eigen::Isometry3d world_to_body_last;
    {
      std::unique_lock<std::mutex> lock(robot_state_mutex_);
      world_to_body_last = world_to_body_msg_;
    }

    // TODO retrieve these from somewhere (TF?)
    body_to_lidar_ = Eigen::Isometry3d::Identity();
    head_to_lidar_ = Eigen::Isometry3d::Identity();

    Eigen::Isometry3d world_to_lidar_now = world_to_body_last * body_to_lidar_;
    world_to_head_now_ = world_to_lidar_now * head_to_lidar_.inverse();

    std::shared_ptr<bot_core::planar_lidar_t> lcm_lidar_msg;
    lcm_lidar_msg->utime = lidar_msg->header.stamp.toNSec() / 1000;
    lcm_lidar_msg->intensities = lidar_msg->intensities;
    lcm_lidar_msg->nintensities = lidar_msg->intensities.size();
    lcm_lidar_msg->nranges = lidar_msg->ranges.size();
    lcm_lidar_msg->rad0 = lidar_msg->angle_min;
    lcm_lidar_msg->radstep = lidar_msg->angle_increment;
    lcm_lidar_msg->ranges = lidar_msg->ranges;


    // 4. Store in LidarScan current scan wrt lidar frame
    LidarScan* current_scan = new LidarScan(lcm_lidar_msg->utime,
                                            lidar_msg->angle_min,
                                            lidar_msg->angle_increment,
                                            lidar_msg->ranges,
                                            lidar_msg->intensities,
                                            world_to_head_now_,
                                            head_to_lidar_,
                                            world_to_body_last);

    // Accumulate current scan in point cloud (projection to default reference "body")
    if (!clear_clouds_buffer_)
    // Accumulate EITHER always OR only when transition from walking to standing happens (DEPRECATED)
    // Pyhton script convert_robot_behavior_type.py must be running.
    {
      if ( accu_->getCounter() % ca_cfg_.batch_size == 0 ) {
        cout << "[Main] " << accu_->getCounter() << " of " << ca_cfg_.batch_size << " scans collected." << endl;
      }
      accu_->processLidar(lcm_lidar_msg);
      lidar_scans_list_.push_back(*current_scan);
    }
    else
    {
      {
        std::unique_lock<std::mutex> lock(cloud_accumulate_mutex_);
        clear_clouds_buffer_ = FALSE;
        cout << "[Main] Cleaning cloud buffer of " << accu_->getCounter() << " scans." << endl;
      }

      if ( accu_->getCounter() > 0 )
        accu_->clearCloud();
      if ( !lidar_scans_list_.empty() )
        lidar_scans_list_.clear();
    }

    delete current_scan;

    if ( accu_->getFinished() ){ //finished accumulating?
      std::cout << "[Main] Finished Collecting: " << accu_->getFinishedTime() << std::endl;

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
      pc_vis_->convertCloudProntoToPcl(*accu_->getCloud(), *cloud);
      // cloud = accu_->getCloud();
      cloud->width = cloud->points.size();
      cloud->height = 1;
      cout << "[Main] Processing cloud with " << cloud->points.size() << " points." << endl;

      // Push this cloud onto the work queue (mutex safe)
      const int max_queue_size = 100;
      {
        std::unique_lock<std::mutex> lock(data_mutex_);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr data (new pcl::PointCloud<pcl::PointXYZRGB>(*cloud));
        data_queue_.push_back(data);
        scans_queue_.push_back(lidar_scans_list_);
        if (data_queue_.size() > max_queue_size) {
          cout << "[Main] WARNING: dropping " <<
            (data_queue_.size()-max_queue_size) << " scans" << endl;
        }
        while (data_queue_.size() > max_queue_size) {
          data_queue_.pop_front();
          scans_queue_.pop_front();
        }
        lidar_scans_list_.clear();
        accu_->clearCloud();
      }
      // Send notification to operator which is waiting for this condition variable
      worker_condition_.notify_one();
    }

}

void AppROS::robotPoseCallBack(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose_msg){

}

void AppROS::run(){
    worker_thread_ = std::thread(std::ref(*this)); // std::ref passes a pointer for you behind the scene

    // Storage
    sweep_scans_list_ = new AlignedSweepsCollection();

    // Accumulator
//    accu_ = new CloudAccumulate(lcm_, ca_cfg_, botparam_, botframes_);

    // Used for: convertCloudProntoToPcl
//    pc_vis_ = new pronto_vis( lcm_->getUnderlyingLCM() );

    // Create debug data folder
    data_directory_path_ << "/tmp/aicp_data";
    const char* path = data_directory_path_.str().c_str();
    boost::filesystem::path dir(path);
    if(boost::filesystem::exists(path))
      boost::filesystem::remove_all(path);
    if(boost::filesystem::create_directory(dir))
      cerr << "AICP debug data directory: " << path << endl;

    // Pose initialization
    cout << "Initialization of robot pose...\n";

    // Instantiate objects
    registr_ = create_registrator(reg_params_);
    overlapper_ = create_overlapper(overlap_params_);
    classifier_ = create_classifier(class_params_);
}

}
