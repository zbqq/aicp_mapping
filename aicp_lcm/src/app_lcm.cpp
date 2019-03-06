#include "aicp_lcm/app_lcm.hpp"

namespace aicp {

AppLCM::AppLCM(boost::shared_ptr<lcm::LCM> &lcm,
               const CommandLineConfig& cl_cfg,
               CloudAccumulateConfig ca_cfg,
               RegistrationParams reg_params,
               OverlapParams overlap_params,
               ClassificationParams class_params) :
    App(cl_cfg, reg_params, overlap_params, class_params),
    ca_cfg_(ca_cfg), lcm_(lcm)
{
    paramInit();

    // Set up frames
    do {
    botparam_ = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 0); // 1 means keep updated, 0 would ignore updates
    } while (botparam_ == NULL);
    botframes_= bot_frames_get_global(lcm_->getUnderlyingLCM(), botparam_);

    // Data structure
    aligned_clouds_graph_ = new AlignedCloudsGraph();
    // Accumulator
    accu_ = new CloudAccumulate(lcm_, ca_cfg_, botparam_, botframes_);
    // Used for: convertCloudProntoToPcl
    pc_vis_ = new pronto_vis(lcm_->getUnderlyingLCM());
    // Visualizer
    vis_ = new LCMVisualizer(lcm_);

    // Thread
    worker_thread_ = std::thread(std::ref(*this)); // std::ref passes a pointer for you behind the scene

    // Laser subsciber
    lcm_->subscribe(ca_cfg_.lidar_channel, &AppLCM::planarLidarHandler, this);
    // Pose subsciber
    lcm_->subscribe(cl_cfg_.pose_body_channel, &AppLCM::robotPoseHandler, this);
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

    // Instantiate objects
    registr_ = create_registrator(reg_params_);
    overlapper_ = create_overlapper(overlap_params_);
    classifier_ = create_classifier(class_params_);
}

void AppLCM::robotPoseHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg){

    Eigen::Isometry3d world_to_body;
    std::unique_lock<std::mutex> lock(robot_state_mutex_);
    {
        // Latest world -> body (pose prior)
        world_to_body_msg_ = getPoseAsIsometry3d(msg);
        world_to_body = world_to_body_msg_;
    }

    bot_core::pose_t msg_out;

    // Compute and publish correction, same frequency as input pose (if "debug" mode)
    if (cl_cfg_.working_mode == "debug")
    {
        // Apply correction if available (identity otherwise)
        // TODO: this could be wrong and must be fixed to match cl_cfg_.working_mode == "robot" case
        corrected_pose_ = total_correction_ * world_to_body;  // world -> reference =
                                                              // body -> reference * world -> body

        // Publish CORRECTED_POSE
        msg_out = getIsometry3dAsBotPose(corrected_pose_, msg->utime);
        lcm_->publish(cl_cfg_.output_channel,&msg_out);

        if (updated_correction_)
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
        // Publish OVERLAP
        bot_core::double_array_t msg_overlap;
        msg_overlap.utime = msg->utime;
        msg_overlap.num_values = 1;
        msg_overlap.values.push_back(octree_overlap_);
        lcm_->publish("OVERLAP",&msg_overlap);

        // Publish ALIGNABILITY
        bot_core::double_array_t msg_al;
        msg_al.utime = msg->utime;
        msg_al.num_values = 1;
        msg_al.values.push_back(alignability_);
        lcm_->publish("ALIGNABILITY",&msg_al);

        if (!risk_prediction_.isZero())
        {
            // Publish ALIGNMENT_RISK
            bot_core::double_array_t msg_risk;
            msg_risk.utime = msg->utime;
            msg_risk.num_values = 1;
            msg_risk.values.push_back(risk_prediction_(0,0));
            lcm_->publish("ALIGNMENT_RISK",&msg_risk);
        }
    }

    pose_initialized_ = TRUE;
}

void AppLCM::planarLidarHandler(const lcm::ReceiveBuffer* rbuf,
                                const std::string& channel,
                                const bot_core::planar_lidar_t* msg)
{
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

    // Accumulate planar scans to 3D point cloud (global frame)
    if (!clear_clouds_buffer_)
    {
        if ( accu_->getCounter() % ca_cfg_.batch_size == 0 ) {
            cout << "[App LCM] " << accu_->getCounter() << " of " << ca_cfg_.batch_size << " scans collected." << endl;
        }
        accu_->processLidar(msg);
    }
    else
    {
        {
            std::unique_lock<std::mutex> lock(cloud_accumulate_mutex_);
            clear_clouds_buffer_ = FALSE;
            cout << "[App LCM] Cleaning cloud buffer of " << accu_->getCounter() << " scans." << endl;
        }

        if ( accu_->getCounter() > 0 )
            accu_->clearCloud();
    }

    if ( accu_->getFinished() ){ //finished accumulating?
        std::cout << "[App LCM] Finished collecting time: " << accu_->getFinishedTime() << std::endl;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pronto (new pcl::PointCloud<pcl::PointXYZRGB> ());
        pc_vis_->convertCloudProntoToPcl(*accu_->getCloud(), *cloud_pronto);
        // cloud_pronto = accu_->getCloud();
        cloud_pronto->width = cloud_pronto->points.size();
        cloud_pronto->height = 1;
        cout << "[App LCM] Processing cloud with " << cloud_pronto->points.size() << " points." << endl;

        // Push this cloud onto the work queue (mutex safe)
        const int max_queue_size = 100;
        {
            std::unique_lock<std::mutex> lock(data_mutex_);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

            pcl::copyPointCloud(*cloud_pronto,*cloud);

            // Populate AlignedCloud data structure
            AlignedCloudPtr current_cloud (new AlignedCloud(msg->utime,
                                                            cloud,
                                                            world_to_body));
            // Stack current cloud into queue
            cloud_queue_.push_back(current_cloud);

            if (cloud_queue_.size() > max_queue_size) {
                cout << "[App LCM] WARNING: dropping " <<
                        (cloud_queue_.size()-max_queue_size) << " clouds." << endl;
            }
            while (cloud_queue_.size() > max_queue_size) {
                cloud_queue_.pop_front();
            }
            accu_->clearCloud();
        }

        // Send notification to operator()() which is waiting for this condition variable
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

//Eigen::Isometry3d AppLCM::getTransfParamAsIsometry3d(PM::TransformationParameters T)
//{
//    Eigen::Isometry3d pose_iso;
//    pose_iso.setIdentity();

//    Eigen::Matrix3f rot;
//    rot << T.block(0,0,3,3);
//    Eigen::Quaternionf quat(rot);
//    Eigen::Quaterniond quatd;
//    quatd = quat.cast <double> ();

//    Eigen::Vector3f transl;
//    transl << T(0,3), T(1,3), T(2,3);
//    Eigen::Vector3d transld;
//    transld = transl.cast <double> ();

//    pose_iso.translation() << transld;
//    Eigen::Quaterniond quatE = Eigen::Quaterniond(quatd.w(), quatd.x(),
//                                                  quatd.y(), quatd.z());
//    pose_iso.rotate(quatE);

//    return pose_iso;
//}

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
