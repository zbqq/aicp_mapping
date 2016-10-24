// sequential-registration (while playing log file)

// Input: POSE_BODY, Output POSE_BODY_CORRECTED
// Computes T_DICP and corrects the estimate from ihmc (POSE_BODY)

// Get path to registration base
#ifdef CONFDIR
  # define REG_BASE CONFDIR
#else
  # define REG_BASE "undefined"
#endif

#include <zlib.h>
#include <lcm/lcm-cpp.hpp>

#include <lcmtypes/bot_core/planar_lidar_t.hpp>
#include <lcmtypes/bot_core/pose_t.hpp>
#include <lcmtypes/bot_core/rigid_transform_t.hpp>
#include <lcmtypes/bot_core/double_array_t.hpp>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <boost/shared_ptr.hpp>
#include <boost/assign/std/vector.hpp>
#include <ConciseArgs>

#include <bot_param/param_client.h>
#include <bot_frames/bot_frames.h>

#include <pcl/io/vtk_io.h>

#include <cloud_accumulate/cloud_accumulate.hpp>
#include <icp-registration/icp_3Dreg_and_plot.hpp>
#include <icp-registration/icp_utils.h>
#include <icp-registration/vtkUtils.h>

#include "timingUtils/timingUtils.hpp"
#include "drawingUtils/drawingUtils.hpp"
#include "filteringUtils/filteringUtils.hpp"

#include "aligned_sweeps_collection.hpp"

using namespace std;

struct CommandLineConfig
{
  std::string robot_name;
  std::string pose_body_channel;
  std::string vicon_channel;
  std::string output_channel;
  std::string working_mode;
  std::string algorithm;
  bool register_if_walking;
  bool apply_correction;
};

class App{
  public:
    App(boost::shared_ptr<lcm::LCM> &lcm_, const CommandLineConfig& cl_cfg_, 
        CloudAccumulateConfig ca_cfg_, RegistrationConfig reg_cfg_);
    
    ~App(){
    }
    
    CloudAccumulateConfig ca_cfg_;
    RegistrationConfig reg_cfg_;

    // thread function for doing actual work
    void operator()();
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    const CommandLineConfig cl_cfg_;
  
    CloudAccumulate* accu_; 
    Registration* registr_;

    // %%% Thread variables %%%
    bool running_;
    std::thread worker_thread_;
    std::condition_variable worker_condition_;
    std::mutex worker_mutex_;
    std::list<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> data_queue_;
    std::list<vector<LidarScan>> scans_queue_;
    std::mutex data_mutex_;
    std::mutex robot_state_mutex_;
    std::mutex robot_behavior_mutex_;
    // %%%%%%%%%%%%%%%%%%%%%%%%

    BotParam* botparam_;
    BotFrames* botframes_;

    // Data structures for storage
    vector<LidarScan> lidar_scans_list_;
    AlignedSweepsCollection* sweep_scans_list_;

    // Transformation matrices
    Eigen::Isometry3d local_;
    Eigen::Isometry3d body_to_lidar_; // Variable tf from the lidar to the robot's base link
    Eigen::Isometry3d head_to_lidar_; // Fixed tf from the lidar to the robot's head frame
    Eigen::Isometry3d world_to_body_msg_; // Captures the position of the body frame in world from launch
                                          // without drift correction
    //Eigen::Isometry3d world_to_body_now_; // running position estimate
    Eigen::Isometry3d world_to_head_now_; // running head position estimate
    Eigen::Isometry3d world_to_frame_vicon_first_; // Ground truth (initialization)
    Eigen::Isometry3d world_to_frame_vicon_; // Ground truth
    Eigen::Isometry3d world_to_body_corr_first_;

    bool pose_initialized_;
    bool updated_correction_;
    bool vicon_initialized_;

    // Robot behavior
    bool accumulate_;
    int robot_behavior_now_;
    int robot_behavior_previous_;

    // visualisation
    pronto_vis* pc_vis_;

    // Overlap parameter
    float overlap_;
    float angularView_;
    // Temporary config file for ICP chain: copied and trimmed ratio replaced
    string tmpConfigName_;
    // Initialize ICP
    PM::TransformationParameters initialT_;
    Eigen::VectorXf x_perturbs_;
    Eigen::VectorXf y_perturbs_;
    Eigen::VectorXf yaw_perturbs_;

    // Correction variables
    Eigen::Isometry3d corrected_pose_;
    Eigen::Isometry3d current_correction_;

    // Init handlers:
    void planarLidarHandler(const lcm::ReceiveBuffer* rbuf, 
                            const std::string& channel, const  bot_core::planar_lidar_t* msg);
    void doRegistration(DP &reference, DP &reading, Eigen::Isometry3d &ref_pose,
                        Eigen::Isometry3d &read_pose, DP &output, PM::TransformationParameters &T);

    void poseInitHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg);
    void viconInitHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::rigid_transform_t* msg);

    // Valkyrie Unit D and Atlas
    void behaviorCallback(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::double_array_t* msg);

    bot_core::pose_t getIsometry3dAsBotPose(Eigen::Isometry3d pose, int64_t utime);
    Eigen::Isometry3d getPoseAsIsometry3d(const bot_core::pose_t* pose);
    Eigen::Isometry3d getBodyAsIsometry3d(const bot_core::rigid_transform_t* pose);
    Eigen::Isometry3d getTransfParamAsIsometry3d(PM::TransformationParameters T);
};    

App::App(boost::shared_ptr<lcm::LCM> &lcm_, const CommandLineConfig& cl_cfg_, 
         CloudAccumulateConfig ca_cfg_, RegistrationConfig reg_cfg_) : lcm_(lcm_), 
         cl_cfg_(cl_cfg_), ca_cfg_(ca_cfg_), reg_cfg_(reg_cfg_){

  pose_initialized_ = FALSE;
  updated_correction_ = TRUE;
  vicon_initialized_ = FALSE;

  accumulate_ = TRUE;
  robot_behavior_now_ = -1;
  robot_behavior_previous_ = -1;

  overlap_ = -1.0;
  if (cl_cfg_.robot_name == "val")
    angularView_ = 180.0;
  else if (cl_cfg_.robot_name == "atlas")
    angularView_ = 220.0;
  else
    angularView_ = 270.0;

  // File used to update config file for ICP chain
  tmpConfigName_.append(REG_BASE);
  tmpConfigName_.append("/filters_config/icp_autotuned_default.yaml");
  // Initialize ICP
  initialT_ = PM::TransformationParameters::Identity(4,4);
  x_perturbs_ = getRandomGaussianVariable(0.0, 0.1, 300);
  y_perturbs_ = getRandomGaussianVariable(0.0, 0.1, 300);
  yaw_perturbs_ = getRandomGaussianVariable(0.0, 10, 300);

  local_ = Eigen::Isometry3d::Identity();
  world_to_body_msg_ = Eigen::Isometry3d::Identity();
  world_to_frame_vicon_ = Eigen::Isometry3d::Identity();
  world_to_frame_vicon_first_ = Eigen::Isometry3d::Identity();
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
  // for Valkyrie Unit D and Atlas
  lcm_->subscribe("CURRENT_ROBOT_BEHAVIOR", &App::behaviorCallback, this);

  // Storage
  sweep_scans_list_ = new AlignedSweepsCollection();

  // Accumulator
  accu_ = new CloudAccumulate(lcm_, ca_cfg_, botparam_, botframes_);

  // Visualiser
  pc_vis_ = new pronto_vis( lcm_->getUnderlyingLCM() );

  // Pose initialization
  lcm_->subscribe(cl_cfg_.pose_body_channel, &App::poseInitHandler, this);
  cout << "Initialization of robot pose...\n";
  lcm_->subscribe(cl_cfg_.vicon_channel, &App::viconInitHandler, this);
  cout << "Initialization of Vicon pose...\n";

  // ICP chain
  registr_ = new Registration(reg_cfg_);
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

void App::doRegistration(DP &reference, DP &reading, Eigen::Isometry3d &ref_pose,
                        Eigen::Isometry3d &read_pose, DP &output, PM::TransformationParameters &T)
{
  // EXPERIMENT: Sensitivity to Input
  // Initial Perturbations: random variables from Gaussian distribution
  PM::TransformationParameters perturbsT = PM::TransformationParameters::Identity(4,4);
  /*std::stringstream perturbs_tmp;
  perturbs_tmp << to_string(x_perturbs_(sweep_scans_list_->getNbClouds()));
  perturbs_tmp << ",";
  perturbs_tmp << to_string(y_perturbs_(sweep_scans_list_->getNbClouds()));
  perturbs_tmp << ",";
  perturbs_tmp << to_string(yaw_perturbs_(sweep_scans_list_->getNbClouds()));
  string perturbs_str;
  perturbs_str.append(perturbs_tmp.str());
  cout << "Initial Perturbation: " << perturbs_str << endl;
  perturbsT = parseTransformationDeg(perturbs_str, 3);*/

  // PRE-FILTERING using filteringUtils
  if (cl_cfg_.algorithm == "aicp")
  {
    //planeModelSegmentationFilter(reference);
    //planeModelSegmentationFilter(reading);
    regionGrowingPlaneSegmentationFilter(reference);
    regionGrowingPlaneSegmentationFilter(reading);
  }

  // Overlap
  DP ref_try, read_try;
  ref_try = reference;
  read_try = reading;
  overlap_ = overlapFilter(ref_try, read_try, ref_pose, read_pose, ca_cfg_.max_range, angularView_);
  cout << "Overlap: " << overlap_ << "%" << endl;
  // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  // To file: DEBUG
  /*std::stringstream vtk_fname;
  vtk_fname << "beforeICP_";
  vtk_fname << to_string(sweep_scans_list_->getNbClouds());
  vtk_fname << ".vtk";
  savePointCloudVTK(vtk_fname.str().c_str(), reading);*/
  // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  // ............do registration.............
  // First ICP loop
  string configName1;
  configName1.append(REG_BASE);
  if (cl_cfg_.algorithm == "icp")
    configName1.append("/filters_config/Chen91_pt2plane.yaml");
  else if (cl_cfg_.algorithm == "aicp")
    configName1.append("/filters_config/icp_autotuned.yaml");

  DP initializedReading;
  // Initialization (if user rquires to apply the correction)
  if (cl_cfg_.apply_correction && cl_cfg_.working_mode != "robot")
  {
    PM::Transformation* rigidTrans;
    rigidTrans = PM::get().REG(Transformation).create("RigidTransformation");

    if (!rigidTrans->checkParameters(initialT_)) {
      cerr << endl
         << "Initial transformation is not rigid. Initialization: identity."
         << endl;
      initialT_ = PM::TransformationParameters::Identity(4,4);
    }
    else
    {
      initialT_ = perturbsT * initialT_;
      initializedReading = rigidTrans->compute(reading, initialT_);
      cout << "Initialization:" << endl << initialT_ << endl;
    }
  }
  else
  {
    initializedReading = reading;
    initialT_ = PM::TransformationParameters::Identity(4,4);
    cout << "Initialization: not applied." << endl;
  }

  // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  // To file: DEBUG
  /*std::stringstream vtk_fname2;
  vtk_fname2 << "afterInitICP_";
  vtk_fname2 << to_string(sweep_scans_list_->getNbClouds());
  vtk_fname2 << ".vtk";
  savePointCloudVTK(vtk_fname2.str().c_str(), initializedReading);*/
  // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  // Auto-tune ICP chain (quantile for the Trimmed Outlier Filter)
  float current_ratio = overlap_/100.0;
  if (current_ratio < 0.25)
    current_ratio = 0.25;
  else if (current_ratio > 0.70)
    current_ratio = 0.70;

  if (cl_cfg_.algorithm == "aicp")
    replaceRatioConfigFile(tmpConfigName_, configName1, current_ratio);

  registr_->setConfigFile(configName1);
  registr_->getICPTransform(initializedReading, reference);
  PM::TransformationParameters T1 = registr_->getTransform();
  cout << "3D Transformation (Trimmed Outlier Filter):" << endl << T1 << endl;
  // Store output after first ICP
  // (if second ICP gives exception the output cloud remains this one)
  output = registr_->getDataOut();

  PM::ICP icp = registr_->getIcp();
  DP readFiltered = icp.getReadingFiltered();

  // To file, registration advanced %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% EVALUATION
  /*
  // Distance dp_cloud points from KNN in ref
  int line_number = sweep_scans_list_->getNbClouds();
  PM::Matrix distsRead = distancesKNN(reference, initializedReading);
  writeLineToFile(distsRead, "distsBeforeRegistration.txt", line_number);
  // Distance out_trimmed_filter points from KNN in ref
  PM::Matrix distsOut1 = distancesKNN(reference, output);
  writeLineToFile(distsOut1, "distsAfterSimpleRegistration.txt", line_number);*/
  //pairedPointsMeanDistance(reference, output, icp);

  // To director
  //drawPointCloudCollections(lcm_, sweep_scans_list_->getNbClouds(), local_, output, 1);
  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  T = T1 * initialT_;
  // Variable to apply correction (initialize the alignment)
  float distT = sqrt(pow(T1(0,3),2) + pow(T1(1,3),2) + pow(T1(2,3),2));
  cout << "distT = " << distT << endl;
  
  if (distT < 0.50)
    initialT_ = T;
  else
    initialT_ = PM::TransformationParameters::Identity(4,4);
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
      DP dp_cloud;
      fromPCLToDataPoints(dp_cloud, *cloud);

      // For storing current cloud
      SweepScan* current_sweep = new SweepScan();
      vector<LidarScan> first_sweep_scans_list = scans_queue.front();
      scans_queue.pop_front();

      // To file
      std::stringstream vtk_fname;
      vtk_fname << "accum_advICP_";
      vtk_fname << to_string(sweep_scans_list_->getNbClouds());
        
      if(sweep_scans_list_->isEmpty())
      {
        current_sweep->populateSweepScan(first_sweep_scans_list, dp_cloud, sweep_scans_list_->getNbClouds());
        sweep_scans_list_->initializeCollection(*current_sweep);

        // To director
        drawPointCloudCollections(lcm_, 0, local_, dp_cloud, 1);
      }
      else
      {
        TimingUtils::tic();

        // AICP: Registration against same reference
        DP ref = sweep_scans_list_->getReference().getCloud();
        // Incremental ICP
        //DP ref = sweep_scans_list_->getCloud(sweep_scans_list_->getNbClouds()-1).getCloud();
        DP out;
        PM::TransformationParameters Ttot;

        //Overlap
        Eigen::Isometry3d ref_pose = sweep_scans_list_->getReference().getBodyPose();
        //Eigen::Isometry3d ref_pose = sweep_scans_list_->getCloud(sweep_scans_list_->getNbClouds()-1).getBodyPose();
        Eigen::Isometry3d read_pose = first_sweep_scans_list.back().getBodyPose();

        this->doRegistration(ref, dp_cloud, ref_pose, read_pose, out, Ttot);

        TimingUtils::toc();

        current_sweep->populateSweepScan(first_sweep_scans_list, out, sweep_scans_list_->getNbClouds());

        // Compute correction to pose estimate:
        current_correction_ = getTransfParamAsIsometry3d(Ttot);
        updated_correction_ = TRUE;

        // Publish corrcted pose
        bot_core::pose_t msg_out;
        Eigen::Isometry3d world_to_body_last;
        std::unique_lock<std::mutex> lock(robot_state_mutex_);
        {
          // Get latest world to body transform available
          world_to_body_last = world_to_body_msg_;
        }

        if (cl_cfg_.working_mode == "robot")
        {
          // Apply correction if available (identity otherwise)
          if (updated_correction_)
          {
            corrected_pose_ = current_correction_ * world_to_body_last;
            updated_correction_ = FALSE;

            // To correct robot drift publish CORRECTED POSE
            msg_out = getIsometry3dAsBotPose(corrected_pose_, current_sweep->getUtimeEnd());
            lcm_->publish(cl_cfg_.output_channel,&msg_out);
          }
        }

        // Ttot is the full transform to move the input on the reference cloud
        // Store current sweep 
        sweep_scans_list_->addSweep(*current_sweep, Ttot);

        // To director
        //drawPointCloudCollections(lcm_, sweep_scans_list_->getNbClouds(), local_, out, 1);
      }
     
      // To file
      /*vtk_fname << ".vtk";
      if(sweep_scans_list_->getNbClouds() % 8 == 0)
        savePointCloudVTK(vtk_fname.str().c_str(), sweep_scans_list_->getCurrentCloud().getCloud());*/
        
      current_sweep->~SweepScan(); 
    }
  }
}

void App::planarLidarHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const bot_core::planar_lidar_t* msg){
    
  if (!pose_initialized_){
    cout << "Estimate not initialized, exiting planarLidarHandler.\n";
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

  // DEBUG: Storage in full sweep structure......%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  // Accumulate current scan in point cloud (projection to default reference "body")
  // only if robot is not walking or stepping
  int robot_behavior_now;
  {
    std::unique_lock<std::mutex> lock(robot_behavior_mutex_);
    robot_behavior_now = robot_behavior_now_;
  }

  if (cl_cfg_.register_if_walking || (robot_behavior_now != 1 && accumulate_))
  // Accumulate EITHER always OR only when transition from walking to standing happens
  // Pyhton script convert_robot_behavior_type.py must be running.
  {
    if ( accu_->getCounter() % ca_cfg_.batch_size == 0 ) {
      cout << accu_->getCounter() << " of " << ca_cfg_.batch_size << " scans collected." << endl;
    }
    accu_->processLidar(msg);
    lidar_scans_list_.push_back(*current_scan);
  }
  else
  {
    if ( accu_->getCounter() > 0 )
      accu_->clearCloud();
      if (!lidar_scans_list_.empty())
        lidar_scans_list_.clear();
  }

  delete current_scan;

  if ( accu_->getFinished() ){ //finished accumulating?
    accumulate_ = FALSE;
    std::cout << "Finished Collecting: " << accu_->getFinishedTime() << std::endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
    pc_vis_->convertCloudProntoToPcl(*accu_->getCloud(), *cloud);
    // cloud = accu_->getCloud();
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cout << "Processing cloud with " << cloud->points.size() << " points." << endl;

    // Push this cloud onto the work queue (mutex safe)
    const int max_queue_size = 100;
    {
      std::unique_lock<std::mutex> lock(data_mutex_);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr data (new pcl::PointCloud<pcl::PointXYZRGB>(*cloud));
      data_queue_.push_back(data);
      scans_queue_.push_back(lidar_scans_list_);
      if (data_queue_.size() > max_queue_size) {
        cout << "WARNING: dropping " <<
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

// Valkyrie Unit D and Atlas
void App::behaviorCallback(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::double_array_t* msg)
{
  std::unique_lock<std::mutex> lock(robot_behavior_mutex_);
  {
    robot_behavior_previous_ = robot_behavior_now_;
    robot_behavior_now_ = msg->values[1];

    if (robot_behavior_now_ != 1 && robot_behavior_previous_ == 1)
    {
      accumulate_ = TRUE;
    }
  }
}

void App::viconInitHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::rigid_transform_t* msg){

  Eigen::Isometry3d body_to_vicon_frame;
  get_trans_with_utime( botframes_, "body_vicon", "vicon_frame", msg->utime, body_to_vicon_frame);

  if (!vicon_initialized_)
  {
    // Get pose ground truth (vicon initialization)
    world_to_frame_vicon_first_ = getBodyAsIsometry3d(msg);
    world_to_frame_vicon_first_ = world_to_frame_vicon_first_ * body_to_vicon_frame;
    vicon_initialized_ = TRUE;
  }

  world_to_frame_vicon_ = getBodyAsIsometry3d(msg);
  world_to_frame_vicon_ = world_to_frame_vicon_ * body_to_vicon_frame;

  bot_core::pose_t msg_out_vic = getIsometry3dAsBotPose(world_to_frame_vicon_, msg->utime);
  lcm_->publish("POSE_VICON",&msg_out_vic);
}

void App::poseInitHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg){

  Eigen::Isometry3d world_to_body_last;
  std::unique_lock<std::mutex> lock(robot_state_mutex_);
  {
    // Update world to body transform (from kinematics msg) 
    world_to_body_msg_ = getPoseAsIsometry3d(msg);
    world_to_body_last = world_to_body_msg_;
  }

  bot_core::pose_t msg_out;

  // Compute and publish correction at same frequency as input pose (if "debug" mode)
  if (cl_cfg_.working_mode == "debug")
  {
    // Apply correction if available (identity otherwise)
    corrected_pose_ = current_correction_ * world_to_body_last;

    // To correct robot drift publish CORRECTED POSE
    msg_out = getIsometry3dAsBotPose(corrected_pose_, msg->utime);
    lcm_->publish(cl_cfg_.output_channel,&msg_out);
  }

  // Publish current overlap
  bot_core::double_array_t msg_overlap;
  msg_overlap.utime = msg->utime;
  msg_overlap.num_values = 1;
  msg_overlap.values.push_back(overlap_);
  lcm_->publish("OVERLAP",&msg_overlap);

  if ( !pose_initialized_ ){
    world_to_body_corr_first_ = corrected_pose_;
  }

  /*
  // Visualization: move poses in Vicon reference frame to compare #########
  // Corrected pose
  Eigen::Isometry3d corr_to_vicon_first;
  corr_to_vicon_first = world_to_frame_vicon_first_ * world_to_body_corr_first_.inverse();

  Eigen::Isometry3d corr_to_vicon_pose;
  corr_to_vicon_pose = corr_to_vicon_first * corrected_pose_;
  // Estimated pose
  Eigen::Isometry3d pose_to_vicon_first;
  pose_to_vicon_first = world_to_frame_vicon_first_ * world_to_body_corr_first_.inverse();

  Eigen::Isometry3d pose_to_vicon_pose;
  pose_to_vicon_pose = pose_to_vicon_first * world_to_body_last;

  // To visualize pose_body and pose corrected wrt Vicon
  bot_core::pose_t msg_out_vic = getIsometry3dAsBotPose(pose_to_vicon_pose, msg->utime);
  lcm_->publish("POSE_BODY_VICON",&msg_out_vic);
  bot_core::pose_t msg_out_corr_vic = getIsometry3dAsBotPose(corr_to_vicon_pose, msg->utime);
  lcm_->publish("POSE_BODY_CORRECTED_VICON",&msg_out_corr_vic);
  // ############################################
  */

  pose_initialized_ = TRUE;
}


int main(int argc, char **argv){
  CommandLineConfig cl_cfg;
  cl_cfg.robot_name = "val";
  cl_cfg.working_mode = "robot";
  cl_cfg.algorithm = "aicp";
  cl_cfg.register_if_walking = TRUE;
  cl_cfg.apply_correction = FALSE;
  cl_cfg.pose_body_channel = "POSE_BODY";
  cl_cfg.vicon_channel = "VICON_pelvis_val";
  cl_cfg.output_channel = "POSE_BODY_CORRECTED"; // Create new channel...

  CloudAccumulateConfig ca_cfg;
  ca_cfg.batch_size = 240; // 240 is about 1 sweep
  ca_cfg.min_range = 0.50; //1.85; // remove all the short range points
  ca_cfg.max_range = 15.0; // we can set up to 30 meters (guaranteed range)
  ca_cfg.lidar_channel ="MULTISENSE_SCAN";
  //ca_cfg.check_local_to_scan_valid = FALSE;

  RegistrationConfig reg_cfg;
  // Load initial transform
  reg_cfg.initTrans_.clear();
  reg_cfg.initTrans_.append("0,0,0"); 

  ConciseArgs parser(argc, argv, "aicp-registration");
  parser.add(cl_cfg.robot_name, "r", "robot_name", "Valkyrie, Atlas or Hyq? (i.e. val, atlas, hyq)");
  parser.add(cl_cfg.working_mode, "s", "working_mode", "Robot or Debug? (i.e. robot or debug)"); //Debug if I want to visualize moving frames in Director
  parser.add(cl_cfg.algorithm, "a", "algorithm", "AICP or ICP? (i.e. aicp or icp)");
  parser.add(cl_cfg.register_if_walking, "w", "register_if_walking", "Compute correction also when robot is walking?");
  parser.add(cl_cfg.apply_correction, "c", "apply_correction", "Initialize ICP with corrected pose? (during debug)");
  parser.add(cl_cfg.pose_body_channel, "p", "pose_body_channel", "Kinematics-inertia pose estimate");
  parser.add(cl_cfg.vicon_channel, "v", "vicon_channel", "Ground truth pose from Vicon");
  parser.add(cl_cfg.output_channel, "o", "output_channel", "Corrected pose");
  parser.add(ca_cfg.lidar_channel, "l", "lidar_channel", "Input message e.g MULTISENSE_SCAN");
  parser.add(ca_cfg.batch_size, "b", "batch_size", "Number of scans per full 3D point cloud (at 5RPM)");
  parser.add(ca_cfg.min_range, "m", "min_range", "Closest accepted lidar range");
  parser.add(ca_cfg.max_range, "M", "max_range", "Furthest accepted lidar range");
  parser.parse();

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  App* app= new App(lcm, cl_cfg, ca_cfg, reg_cfg);

  while(0 == lcm->handle());
}
