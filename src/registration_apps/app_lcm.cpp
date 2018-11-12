#include "registration_apps/app_lcm.hpp"
#include "aicp_registration/registration.hpp"
#include "aicp_overlap/overlap.hpp"
#include "aicp_classification/classification.hpp"
#include <lcmtypes/bot_core/double_array_t.hpp>
#include <bot_frames_cpp/bot_frames_cpp.hpp>

namespace aicp {

AppLCM::AppLCM(boost::shared_ptr<lcm::LCM> &lcm,
               const CommandLineConfig& cl_cfg,
               CloudAccumulateConfig ca_cfg,
               RegistrationParams reg_params,
               OverlapParams overlap_params,
               ClassificationParams class_params,
               string exp_params) :
    App(cl_cfg, reg_params, overlap_params, class_params, exp_params),
    ca_cfg_(ca_cfg),
    lcm_(lcm)
{
    paramInit();

  // Set up frames and config:
  do {
    botparam_ = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 0); // 1 means keep updated, 0 would ignore updates
  } while (botparam_ == NULL);
  botframes_= bot_frames_get_global(lcm_->getUnderlyingLCM(), botparam_);

  worker_thread_ = std::thread(std::ref(*this)); // std::ref passes a pointer for you behind the scene

  lcm_->subscribe(ca_cfg_.lidar_channel, &AppLCM::planarLidarHandler, this);

  // Storage
  sweep_scans_list_ = new AlignedSweepsCollection();

  // Accumulator
  accu_ = new CloudAccumulate(lcm_, ca_cfg_, botparam_, botframes_);

  // Used for: convertCloudProntoToPcl
  pc_vis_ = new pronto_vis( lcm_->getUnderlyingLCM() );

  // Create debug data folder
  data_directory_path_ << "/tmp/aicp_data";
  const char* path = data_directory_path_.str().c_str();
  boost::filesystem::path dir(path);
  if(boost::filesystem::exists(path))
    boost::filesystem::remove_all(path);
  if(boost::filesystem::create_directory(dir))
    cerr << "AICP debug data directory: " << path << endl;

  // Pose initialization
  lcm_->subscribe(cl_cfg_.pose_body_channel, &AppLCM::poseInitHandler, this);
  cout << "Initialization of robot pose...\n";

  // Instantiate objects
  registr_ = create_registrator(reg_params_);
  overlapper_ = create_overlapper(overlap_params_);
  classifier_ = create_classifier(class_params_);
}

void AppLCM::poseInitHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg){

  Eigen::Isometry3d world_to_body_last;
  std::unique_lock<std::mutex> lock(robot_state_mutex_);
  {
    // Update world to body transform (from kinematics msg)
    world_to_body_msg_ = getPoseAsIsometry3d(msg);
    world_to_body_msg_utime_ = msg->utime;
    world_to_body_last = world_to_body_msg_;
  }

  bot_core::pose_t msg_out;

  // Compute and publish correction at same frequency as input pose (if "debug" mode)
  if (cl_cfg_.working_mode == "debug")
  {
    // Apply correction if available (identity otherwise)
    // TODO: this could be wrong and must be fixed to match cl_cfg_.working_mode == "robot" case
    corrected_pose_ = current_correction_ * world_to_body_last;

    // To correct robot drift publish CORRECTED POSE
    msg_out = getIsometry3dAsBotPose(corrected_pose_, msg->utime);
    lcm_->publish(cl_cfg_.output_channel,&msg_out);

    if (updated_correction_ || rejected_correction)
    {
      if (exp_params_ == "Online" && !risk_prediction_.isZero() && !other_predictions_.empty())
      {
        stringstream ss;
        ss << data_directory_path_.str();
        ss << "/online_results.txt";
        online_results_line_++;
        Eigen::MatrixXf line_elements(1,6);
        float tstamp_to_sec = (msg->utime - world_to_body_msg_utime_first_)/1000000.0;
        line_elements << tstamp_to_sec, octree_overlap_, alignability_, risk_prediction_.cast<float>(),
                         other_predictions_.at(0), other_predictions_.at(1); // degeneracy and ICN respectively
        writeLineToFile(line_elements, ss.str(), online_results_line_);
      }

      if (updated_correction_)
      {
        {
          std::unique_lock<std::mutex> lock(cloud_accumulate_mutex_);
          clear_clouds_buffer_ = TRUE;
        }
      }

      updated_correction_ = FALSE;
      rejected_correction = FALSE;
    }
  }

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

  if ( !pose_initialized_ ){
    world_to_body_corr_first_ = corrected_pose_;
    world_to_body_msg_utime_first_ = msg->utime;
  }

//  /*
//  // Visualization: move poses in Vicon reference frame to compare #########
//  // Corrected pose
//  Eigen::Isometry3d corr_to_vicon_first;
//  corr_to_vicon_first = world_to_frame_vicon_first_ * world_to_body_corr_first_.inverse();

//  Eigen::Isometry3d corr_to_vicon_pose;
//  corr_to_vicon_pose = corr_to_vicon_first * corrected_pose_;
//  // Estimated pose
//  Eigen::Isometry3d pose_to_vicon_first;
//  pose_to_vicon_first = world_to_frame_vicon_first_ * world_to_body_corr_first_.inverse();

//  Eigen::Isometry3d pose_to_vicon_pose;
//  pose_to_vicon_pose = pose_to_vicon_first * world_to_body_last;

//  // To visualize pose_body and pose corrected wrt Vicon
//  bot_core::pose_t msg_out_vic = getIsometry3dAsBotPose(pose_to_vicon_pose, msg->utime);
//  lcm_->publish("POSE_BODY_VICON",&msg_out_vic);
//  bot_core::pose_t msg_out_corr_vic = getIsometry3dAsBotPose(corr_to_vicon_pose, msg->utime);
//  lcm_->publish("POSE_BODY_CORRECTED_VICON",&msg_out_corr_vic);
//  // ############################################
//  */

  pose_initialized_ = TRUE;
}

void AppLCM::planarLidarHandler(const lcm::ReceiveBuffer* rbuf,
                                const std::string& channel,
                                const bot_core::planar_lidar_t* msg)
{
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
  // Populate SweepScan with current LidarScan data structure
  // 2. Get lidar pose
  // bot_frames_structure,from_frame,to_frame,utime,result
  getTransWithMicroTime(botframes_, (ca_cfg_.lidar_channel).c_str(), "body", msg->utime, body_to_lidar_);
  getTransWithMicroTime(botframes_, (ca_cfg_.lidar_channel).c_str(), "head", msg->utime, head_to_lidar_);
  // 3. Compute current pose of head in world reference frame
  Eigen::Isometry3d world_to_lidar_now = world_to_body_last * body_to_lidar_;
  world_to_head_now_ = world_to_lidar_now * head_to_lidar_.inverse();
  // 4. Store in LidarScan current scan wrt lidar frame
  LidarScan* current_scan = new LidarScan(msg->utime,msg->rad0,msg->radstep,
                                          msg->ranges,msg->intensities,world_to_head_now_,head_to_lidar_,
                                          world_to_body_last);

  // Accumulate current scan in point cloud (projection to default reference "body")
  if (!clear_clouds_buffer_)
  // Accumulate EITHER always OR only when transition from walking to standing happens (DEPRECATED)
  // Pyhton script convert_robot_behavior_type.py must be running.
  {
    if ( accu_->getCounter() % ca_cfg_.batch_size == 0 ) {
      cout << "[Main] " << accu_->getCounter() << " of " << ca_cfg_.batch_size << " scans collected." << endl;
    }
    accu_->processLidar(msg);
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


bot_core::pose_t AppLCM::getIsometry3dAsBotPose(Eigen::Isometry3d pose, int64_t utime){
  bot_core::pose_t tf;
  tf.utime =   utime;
  tf.pos[0] = pose.translation().x();
  tf.pos[1] = pose.translation().y();
  tf.pos[2] = pose.translation().z();
  Eigen::Quaterniond r_x(pose.rotation());
  tf.orientation[0] =  r_x.w();
  tf.orientation[1] =  r_x.x();
  tf.orientation[2] =  r_x.y();
  tf.orientation[3] =  r_x.z();
  return tf;
}

Eigen::Isometry3d AppLCM::getPoseAsIsometry3d(const bot_core::pose_t* pose){
  Eigen::Isometry3d pose_iso;
  pose_iso.setIdentity();
  pose_iso.translation()  <<  pose->pos[0], pose->pos[1] , pose->pos[2];
  Eigen::Quaterniond quat = Eigen::Quaterniond(pose->orientation[0], pose->orientation[1],
                                                pose->orientation[2], pose->orientation[3]);
  pose_iso.rotate(quat);

  return pose_iso;
}

Eigen::Isometry3d AppLCM::getBodyAsIsometry3d(const bot_core::rigid_transform_t* pose){
  Eigen::Isometry3d pose_iso;
  pose_iso.setIdentity();
  pose_iso.translation()  <<  pose->trans[0], pose->trans[1] , pose->trans[2];
  Eigen::Quaterniond quatern = Eigen::Quaterniond(pose->quat[0], pose->quat[1],
                                                pose->quat[2], pose->quat[3]);
  pose_iso.rotate(quatern);

  return pose_iso;
}

Eigen::Isometry3d AppLCM::getTransfParamAsIsometry3d(PM::TransformationParameters T)
{
  Eigen::Isometry3d pose_iso;
  pose_iso.setIdentity();

  Eigen::Matrix3f rot;
  rot << T.block(0,0,3,3);
  Eigen::Quaternionf quat(rot);
  Eigen::Quaterniond quatd;
  quatd = quat.cast <double> ();

  Eigen::Vector3f transl;
  transl << T(0,3), T(1,3), T(2,3);
  Eigen::Vector3d transld;
  transld = transl.cast <double> ();

  pose_iso.translation() << transld;
  Eigen::Quaterniond quatE = Eigen::Quaterniond(quatd.w(), quatd.x(),
                                                quatd.y(), quatd.z());
  pose_iso.rotate(quatE);

  return pose_iso;
}

int AppLCM::getTransWithMicroTime(BotFrames *bot_frames,
                                 const char *from_frame,
                                 const char *to_frame,
                                 const int64_t& utime,
                                 Eigen::Isometry3d & mat)
{
    int status;
    double matx[16];
    status = bot_frames_get_trans_mat_4x4_with_utime( bot_frames, from_frame,  to_frame, utime, matx);
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        mat(i,j) = matx[i*4+j];
      }
    }

    return status;
}
} // namespace aicp
