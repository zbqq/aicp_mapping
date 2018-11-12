#include "app.hpp"
#include "aicp_common_utils/timing.hpp"
#include "aicp_registration/registration.hpp"
#include "aicp_overlap/overlap.hpp"
#include "aicp_classification/classification.hpp"
#include "aicp_common_utils/common.hpp"
#include "aicp_drawing_utils/drawingUtils.hpp"

#include <lcmtypes/bot_core/double_array_t.hpp>


namespace aicp {
App::App(boost::shared_ptr<lcm::LCM> &lcm_, const CommandLineConfig& cl_cfg_,
         CloudAccumulateConfig ca_cfg_,
         RegistrationParams reg_params_, OverlapParams overlap_params_,
         ClassificationParams class_params_, string exp_params_) :
         lcm_(lcm_), cl_cfg_(cl_cfg_), ca_cfg_(ca_cfg_),
         reg_params_(reg_params_), overlap_params_(overlap_params_),
         class_params_(class_params_), exp_params_(exp_params_){

  pose_initialized_ = FALSE;
  updated_correction_ = TRUE;
  rejected_correction = TRUE;

  valid_correction_ = FALSE;
  force_reference_update_ = FALSE;

  // Reference cloud update counters
  updates_counter_ = 0;

  // Count lines output file
  online_results_line_ = 0;

  clear_clouds_buffer_ = FALSE;

  fov_overlap_ = -1.0;
  octree_overlap_ = -1.0;
  alignability_ = -1.0;

  // Initialize reading with previous correction when "debug" mode
  initialT_ = Eigen::Matrix4f::Identity(4,4);

  local_ = Eigen::Isometry3d::Identity();
  world_to_body_msg_ = Eigen::Isometry3d::Identity();
  world_to_body_corr_first_ = Eigen::Isometry3d::Identity();
  corrected_pose_ = Eigen::Isometry3d::Identity();
  current_correction_ = Eigen::Isometry3d::Identity();

  // Set up frames and config:
  do {
    botparam_ = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 0); // 1 means keep updated, 0 would ignore updates
  } while (botparam_ == NULL);
  botframes_= bot_frames_get_global(lcm_->getUnderlyingLCM(), botparam_);

  worker_thread_ = std::thread(std::ref(*this)); // std::ref passes a pointer for you behind the scene

  lcm_->subscribe(ca_cfg_.lidar_channel, &App::planarLidarHandler, this);

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
  lcm_->subscribe(cl_cfg_.pose_body_channel, &App::poseInitHandler, this);
  cout << "Initialization of robot pose...\n";

  // Instantiate objects
  registr_ = create_registrator(reg_params_);
  overlapper_ = create_overlapper(overlap_params_);
  classifier_ = create_classifier(class_params_);
}

int get_trans_with_utime(BotFrames *bot_frames,
        const char *from_frame, const char *to_frame, int64_t utime,
        Eigen::Isometry3d & mat){
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

bot_core::pose_t App::getIsometry3dAsBotPose(Eigen::Isometry3d pose, int64_t utime){
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

Eigen::Isometry3d App::getPoseAsIsometry3d(const bot_core::pose_t* pose){
  Eigen::Isometry3d pose_iso;
  pose_iso.setIdentity();
  pose_iso.translation()  <<  pose->pos[0], pose->pos[1] , pose->pos[2];
  Eigen::Quaterniond quat = Eigen::Quaterniond(pose->orientation[0], pose->orientation[1],
                                                pose->orientation[2], pose->orientation[3]);
  pose_iso.rotate(quat);

  return pose_iso;
}

Eigen::Isometry3d App::getBodyAsIsometry3d(const bot_core::rigid_transform_t* pose){
  Eigen::Isometry3d pose_iso;
  pose_iso.setIdentity();
  pose_iso.translation()  <<  pose->trans[0], pose->trans[1] , pose->trans[2];
  Eigen::Quaterniond quatern = Eigen::Quaterniond(pose->quat[0], pose->quat[1],
                                                pose->quat[2], pose->quat[3]);
  pose_iso.rotate(quatern);

  return pose_iso;
}

Eigen::Isometry3d App::getTransfParamAsIsometry3d(PM::TransformationParameters T){
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

void App::doRegistration(pcl::PointCloud<pcl::PointXYZ>& reference, pcl::PointCloud<pcl::PointXYZ>& reading,
                         Eigen::Matrix4f &T, vector<float>& failure_prediction_factors)
{
  /*===================================
  =              AICP Core            =
  ===================================*/
  string configNameAICP;
  configNameAICP.append(FILTERS_CONFIG_LOC);
  configNameAICP.append("/icp_autotuned.yaml");

  // Auto-tune ICP chain (quantile for the outlier filter)
  float current_ratio = octree_overlap_/100.0;
  if (current_ratio < 0.25)
    current_ratio = 0.25;
  else if (current_ratio > 0.70)
    current_ratio = 0.70;

  replaceRatioConfigFile(reg_params_.pointmatcher.configFileName, configNameAICP, current_ratio);
  registr_->updateConfigParams(configNameAICP);

  /*===================================
  =          Register Clouds          =
  ===================================*/
  registr_->registerClouds(reference, reading, T, failure_prediction_factors);
  if (!failure_prediction_factors.empty())
  {
    cout << "[Main] Degeneracy (degenerate if ~ 0): " << failure_prediction_factors.at(0) << " %" << endl;
    cout << "[Main] ICN (degenerate if ~ 0): " << failure_prediction_factors.at(1) << endl;
  }

  cout << "==============================" << endl
       << "[Main] Computed 3D Transform:" << endl
       << "==============================" << endl
       << T << endl;

  T =  T * initialT_;

  cout << "==============================" << endl
       << "[Main] Corrected Reading Pose:" << endl
       << "==============================" << endl
       << T << endl;
}

void App::operator()() {
  running_ = true;
  while (running_) {
    std::unique_lock<std::mutex> lock(worker_mutex_);
    worker_condition_.wait_for(lock, std::chrono::milliseconds(1000));

    // copy current workload from data queue to work queue
    std::list<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> work_queue;
    std::list<vector<LidarScan>> scans_queue;
    {
      std::unique_lock<std::mutex> lock(data_mutex_);
      while (!data_queue_.empty()) {
        work_queue.push_back(data_queue_.front());
        data_queue_.pop_front();
        scans_queue.push_back(scans_queue_.front());
        scans_queue_.pop_front();
      }
    }

    // process workload
    for (auto cloud : work_queue) {
      // For storing current cloud
      SweepScan* current_sweep = new SweepScan();
      vector<LidarScan> first_sweep_scans_list = scans_queue.front();
      scans_queue.pop_front();

      if(sweep_scans_list_->isEmpty())
      {
        current_sweep->populateSweepScan(first_sweep_scans_list, *cloud, sweep_scans_list_->getNbClouds());
        sweep_scans_list_->initializeCollection(*current_sweep);

        if (cl_cfg_.verbose)
        {
          // To director first reference cloud (Point Cloud 0, Frame 0)
          drawPointCloudCollections(lcm_, 0, local_, *cloud, 1);
        }

        // To file
        stringstream ss_ref;
        ss_ref << data_directory_path_.str();
        ss_ref << "/ref_";
        ss_ref << to_string(0);
        ss_ref << ".pcd";
        pcd_writer_.write<pcl::PointXYZRGB> (ss_ref.str (), *cloud, false);
      }
      else
      {
        TimingUtils::tic();

        if (cl_cfg_.verbose)
        {
          // Publish clouds in Director
          drawPointCloudCollections(lcm_, 5000, local_, *cloud, 1, "Reading Original");
        }

        /*===================================
        =           Set Input Clouds        =
        ===================================*/
        // Get current reference cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr ref = sweep_scans_list_->getCurrentReference().getCloud();

        Eigen::Isometry3d ref_pose, read_pose;
        ref_pose = sweep_scans_list_->getCurrentReference().getBodyPose();
        read_pose = first_sweep_scans_list.back().getBodyPose();

        // Initialize clouds before sending to filters
        // (simulates correction integration)
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr ref_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr read_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
        if (cl_cfg_.apply_correction && cl_cfg_.working_mode != "robot")
        {
          pcl::transformPointCloud (*cloud, *read_ptr, initialT_);
          Eigen::Isometry3d initialT_iso = fromMatrix4fToIsometry3d(initialT_);
          read_pose = initialT_iso * read_pose;
        }
        else
          *read_ptr = *cloud;

        ref_ptr = ref;

        if (cl_cfg_.verbose)
        {
          // Publish clouds in Director
          drawPointCloudCollections(lcm_, 5010, local_, *read_ptr, 1, "Reading Initialized");
          drawPointCloudCollections(lcm_, 5020, local_, *ref_ptr, 1, "Reference");
        }

        /*===================================
        =        Filter Input Clouds        =
        ===================================*/

        pcl::PointCloud<pcl::PointXYZ>::Ptr ref_xyz_ptr (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr read_xyz_ptr (new pcl::PointCloud<pcl::PointXYZ>);
        copyPointCloud(*ref_ptr, *ref_xyz_ptr);
        copyPointCloud(*read_ptr, *read_xyz_ptr);

        pcl::PointCloud<pcl::PointXYZ> overlap_points_A;
        pcl::PointCloud<pcl::PointXYZ> overlap_points_B;
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudA_matched_planes (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudB_matched_planes (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr eigenvectors (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

        // ------------------
        // FOV-based Overlap
        // ------------------
        fov_overlap_ = overlapFilter(*ref_xyz_ptr, *read_xyz_ptr,
                                     ref_pose, read_pose,
                                     reg_params_.sensorRange , reg_params_.sensorAngularView,
                                     overlap_points_A, overlap_points_B);
        cout << "====================================" << endl
             << "[Main] FOV-based Overlap: " << fov_overlap_ << " %" << endl
             << "====================================" << endl;

        // ------------------------------------
        // Pre-filtering: 1) down-sampling
        //                2) planes extraction
        // ------------------------------------
        pcl::PointCloud<pcl::PointXYZ>::Ptr ref_xyz_prefiltered (new pcl::PointCloud<pcl::PointXYZ>);
        regionGrowingUniformPlaneSegmentationFilter(ref_xyz_ptr, ref_xyz_prefiltered);
        pcl::PointCloud<pcl::PointXYZ>::Ptr read_xyz_prefiltered (new pcl::PointCloud<pcl::PointXYZ>);
        regionGrowingUniformPlaneSegmentationFilter(read_xyz_ptr, read_xyz_prefiltered);

        stringstream ss_tmp1;
        ss_tmp1 << data_directory_path_.str();
        ss_tmp1 << "/point_cloud_A_prefiltered.pcd";
        pcd_writer_.write<pcl::PointXYZ> (ss_tmp1.str (), *ref_xyz_prefiltered, false);
        stringstream ss_tmp2;
        ss_tmp2 << data_directory_path_.str();
        ss_tmp2 << "/point_cloud_B_prefiltered.pcd";
        pcd_writer_.write<pcl::PointXYZ> (ss_tmp2.str (), *read_xyz_prefiltered, false);

        // ---------------------
        // Octree-based Overlap
        // ---------------------
        ColorOcTree* ref_tree;
        ColorOcTree* read_tree = new ColorOcTree(overlap_params_.octree_based.octomapResolution);

        // Create octree from reference cloud (wrt robot point of view),
        // add the reading cloud and compute overlap
        ref_tree = overlapper_->computeOverlap(*ref_xyz_prefiltered, *read_xyz_prefiltered,
                                               ref_pose, read_pose,
                                               read_tree);
        octree_overlap_ = overlapper_->getOverlap();

        cout << "====================================" << endl
             << "[Main] Octree-based Overlap: " << octree_overlap_ << " %" << endl
             << "====================================" << endl;

        // -------------
        // Alignability
        // -------------
        // Alignability computed on points belonging to the region of overlap (overlap_points_A, overlap_points_B)
        alignability_ = alignabilityFilter(overlap_points_A, overlap_points_B,
                                           ref_pose, read_pose,
                                           cloudA_matched_planes, cloudB_matched_planes, eigenvectors);
        cout << "[Main] Alignability (degenerate if ~ 0): " << alignability_ << " %" << endl;

        /*===================================
        =           Classification          =
        ===================================*/
        // ---------------
        // Alignment Risk
        // ---------------
        MatrixXd testing_data(1, 2);
        testing_data << (float)octree_overlap_, (float)alignability_;

        classifier_->test(testing_data, &risk_prediction_);
        std::cout << "[Main] Alignment Risk Prediction (0-1): " << risk_prediction_ << std::endl;

        if (cl_cfg_.verbose)
        {
          /*===================================
          =     Visualize Octree Overlap      =
          ===================================*/
//          publishOctreeToLCM(lcm_, ref_tree, "OCTOMAP_REF");
//          publishOctreeToLCM(lcm_, read_tree, "OCTOMAP");
          /*===================================
          =      Visualize Alignability       =
          ===================================*/
//          drawPointCloudNormalsCollections(lcm_, 9, local_, *cloudA_matched_planes, 0, "Matches A");
//          drawPointCloudNormalsCollections(lcm_, 11, local_, *cloudB_matched_planes, 0, "Matches B");
//          eigenvectors->points.resize(4);
//          drawPointCloudNormalsCollections(lcm_, 13, local_, *eigenvectors, 0, "Alignability Eigenvectors");
        }

        /*===================================
        =          Register Clouds          =
        ===================================*/
        Eigen::Matrix4f Ttot = Eigen::Matrix4f::Identity(4,4);

        this->doRegistration(*ref_xyz_prefiltered, *read_xyz_prefiltered,
                             Ttot, other_predictions_);

        if((cl_cfg_.failure_prediction_mode == 0 && risk_prediction_(0,0) > class_params_.svm.threshold) ||
           (cl_cfg_.failure_prediction_mode == 1 && other_predictions_.at(0) < 0.05))
        {
            cout << "====================================" << endl
                 << "[Main] REFERENCE UPDATE" << endl
                 << "====================================" << endl;
          // Reference Update Statistics
          updates_counter_ ++;

          int current_cloud_id = sweep_scans_list_->getCurrentCloud().getId();
          // Updating reference with current reading (non-aligned)
          if(sweep_scans_list_->getCloud(current_cloud_id).setReference())
          {
            sweep_scans_list_->getCloud(current_cloud_id).disableReference();
            cout << "SET REFERENCE aligned: " << current_cloud_id << endl;
          }
          else
          {
            current_sweep->populateSweepScan(first_sweep_scans_list, *read_ptr, sweep_scans_list_->getNbClouds(), -1, 1);
            sweep_scans_list_->addSweep(*current_sweep, initialT_);
            current_cloud_id = sweep_scans_list_->getCurrentCloud().getId();
            cout << "SET REFERENCE original: " << current_cloud_id << endl;
          }

          sweep_scans_list_->getCloud(current_cloud_id).setReference();
          sweep_scans_list_->updateReference(current_cloud_id);

          // To file
          stringstream ss_tmp3;
          ss_tmp3 << data_directory_path_.str();
          ss_tmp3 << "/ref_";
          ss_tmp3 << to_string(sweep_scans_list_->getCurrentReference().getId());
          ss_tmp3 << ".pcd";
          pcd_writer_.write<pcl::PointXYZRGB> (ss_tmp3.str (), *sweep_scans_list_->getCurrentReference().getCloud(), false);

          rejected_correction = TRUE;
        }
        else
        {
          bool enableRef = TRUE;
          pcl::PointCloud<pcl::PointXYZRGB>::Ptr output (new pcl::PointCloud<pcl::PointXYZRGB>);
          pcl::transformPointCloud (*cloud, *output, Ttot);
          current_sweep->populateSweepScan(first_sweep_scans_list, *output, sweep_scans_list_->getNbClouds(), sweep_scans_list_->getCurrentReference().getId(), enableRef);
          sweep_scans_list_->addSweep(*current_sweep, Ttot);

          current_correction_ = getTransfParamAsIsometry3d(Ttot);
          updated_correction_ = TRUE;

          initialT_ = Ttot;
        }

        TimingUtils::toc();

        // DEBUG
        cout << "============================" << endl
             << "[Main] Statistics:" << endl
             << "============================" << endl;
        cout << "REFERENCE: " << sweep_scans_list_->getCurrentReference().getId() << endl;
        cout << "Cloud ID: " << sweep_scans_list_->getCurrentCloud().getId() << endl;
        cout << "Number Clouds: " << sweep_scans_list_->getNbClouds() << endl;
        cout << "Updates: " << updates_counter_ << endl;
      }

      if (cl_cfg_.verbose)
        drawPointCloudCollections(lcm_, 5030, local_, *sweep_scans_list_->getCurrentCloud().getCloud(), 1, "Reading Aligned");

      current_sweep->~SweepScan();

      cout << "--------------------------------------------------------------------------------------" << endl;
    }
  }
}

void App::planarLidarHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const bot_core::planar_lidar_t* msg){
    
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
  get_trans_with_utime( botframes_, (ca_cfg_.lidar_channel).c_str(), "body", msg->utime, body_to_lidar_);
  get_trans_with_utime( botframes_, (ca_cfg_.lidar_channel).c_str(), "head", msg->utime, head_to_lidar_);
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

void App::poseInitHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg){

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
} // namespace aicp
