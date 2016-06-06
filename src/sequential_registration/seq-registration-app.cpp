// sequential-registration (while playing log file)

// Input: POSE_BODY, Output POSE_BODY_CORRECTED
// Computes T_DICP and corrects the estimate from ihmc (POSE_BODY)
// --> POSE_BODY = POSE_BODY * T_DICP.inverse() 

#include <zlib.h>
#include <lcm/lcm-cpp.hpp>

#include <lcmtypes/bot_core.hpp>
#include <lcmtypes/drc/behavior_t.hpp>
#include <lcmtypes/drc/controller_status_t.hpp>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <boost/shared_ptr.hpp>
#include <boost/assign/std/vector.hpp>
#include <ConciseArgs>

#include <bot_param/param_client.h>
#include <bot_frames/bot_frames.h>

#include <pcl/io/vtk_io.h>

#include <icp-registration/cloud_accumulate.hpp>
#include <icp-registration/icp_3Dreg_and_plot.hpp>
#include <icp-registration/icp_utils.h>
#include <icp-registration/vtkUtils.h>

#include "drawingUtils/drawingUtils.hpp"
#include "timingUtils/timingUtils.hpp"
#include "aligned_sweeps_collection.hpp"

using namespace std;

struct CommandLineConfig
{
  std::string robot_name;
  bool init_with_message; // initialize off of a pose or vicon
  std::string output_channel;
};

struct IsometryTime3d{
  IsometryTime3d(int64_t utime, const Eigen::Isometry3d & pose) : utime(utime), pose(pose) {}
    int64_t utime;
    Eigen::Isometry3d pose;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class App{
  public:
    App(boost::shared_ptr<lcm::LCM> &lcm_, const CommandLineConfig& cl_cfg_, 
        CloudAccumulateConfig ca_cfg_, RegistrationConfig reg_cfg_);
    
    ~App(){
    }
    
    CloudAccumulateConfig ca_cfg_;
    RegistrationConfig reg_cfg_;

    void plotBotFrame(const char* from_frame, const char* to_frame, int64_t utime);

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
    FrameCheckTools frame_check_tools_;

    // Data structures for storage
    vector<LidarScan> lidar_scans_list_;
    AlignedSweepsCollection* sweep_scans_list_;

    // Transformation matrices
    Eigen::Isometry3d body_to_lidar_; // Variable tf from the lidar to the robot's base link
    Eigen::Isometry3d head_to_lidar_; // Fixed tf from the lidar to the robot's head frame
    IsometryTime3d world_to_body_msg_; // Captures the position of the body frame in world from launch 
                                          // without drift correction
    //Eigen::Isometry3d world_to_body_now_; // running position estimate
    Eigen::Isometry3d world_to_head_now_; // running head position estimate

    bool pose_initialized_;

    // Robot behavior
    bool accumulate_;
    int robot_behavior_now_;
    int robot_behavior_previous_;

    // Init handlers:
    void planarLidarHandler(const lcm::ReceiveBuffer* rbuf, 
                            const std::string& channel, const  bot_core::planar_lidar_t* msg);
    void doRegistration(DP &reference, DP &reading, DP &output, PM::TransformationParameters &T);

    void rigidTransformInitHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::rigid_transform_t* msg);
    void poseInitHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg);
    void initState( const double trans[3], const double quat[4], long long int t);

    // Valkyrie
    void behaviorCallbackValkyrie(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::behavior_t* msg);
    // Atlas
    void behaviorCallbackAtlas(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::controller_status_t* msg);

    bot_core::pose_t getIsometry3dAsBotPose(Eigen::Isometry3d pose, int64_t utime);
    IsometryTime3d getPoseAsIsometryTime3d(const double trans[3], const double quat[4], long long int t);
    Eigen::Isometry3d getTransfParamAsIsometry3d(PM::TransformationParameters T);
};    

App::App(boost::shared_ptr<lcm::LCM> &lcm_, const CommandLineConfig& cl_cfg_, 
         CloudAccumulateConfig ca_cfg_, RegistrationConfig reg_cfg_) : lcm_(lcm_), 
         cl_cfg_(cl_cfg_), ca_cfg_(ca_cfg_),
         reg_cfg_(reg_cfg_), world_to_body_msg_(0,Eigen::Isometry3d::Identity()){
  pose_initialized_ = FALSE;

  accumulate_ = TRUE;
  robot_behavior_now_ = -1;
  robot_behavior_previous_ = -1;

  // Set up frames and config:
  do {
    botparam_ = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 0); // 1 means keep updated, 0 would ignore updates
  } while (botparam_ == NULL);
  botframes_= bot_frames_get_global(lcm_->getUnderlyingLCM(), botparam_);

  worker_thread_ = std::thread(std::ref(*this)); // std::ref passes a pointer for you behind the scene

  lcm_->subscribe(ca_cfg_.lidar_channel, &App::planarLidarHandler, this);
  // for Valkyrie Unit D
  lcm_->subscribe("ROBOT_BEHAVIOR", &App::behaviorCallbackValkyrie, this);
  // for Valkyrie January 2016
  lcm_->subscribe("ATLAS_BEHAVIOR", &App::behaviorCallbackValkyrie, this);
  // for Atlas
  lcm_->subscribe("CONTROLLER_STATUS", &App::behaviorCallbackAtlas, this);

  // Storage
  sweep_scans_list_ = new AlignedSweepsCollection();

  // Accumulator
  accu_ = new CloudAccumulate(lcm_, ca_cfg_, botparam_, botframes_);

  // Pose initialization
  lcm_->subscribe("POSE_BODY", &App::poseInitHandler, this); 
  cout << "Initialization of LIDAR pose...\n";  

  // ICP chain
  registr_ = new Registration(lcm_, reg_cfg_);
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

IsometryTime3d App::getPoseAsIsometryTime3d(const double trans[3], const double quat[4], long long int t){
  Eigen::Isometry3d pose_iso;
  pose_iso.setIdentity();
  pose_iso.translation()  << trans[0], trans[1] , trans[2];
  Eigen::Quaterniond quatE = Eigen::Quaterniond(quat[0], quat[1], 
                                                 quat[2], quat[3]);
  pose_iso.rotate(quatE);

  return IsometryTime3d(t, pose_iso);
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

void App::plotBotFrame(const char* from_frame, const char* to_frame, int64_t utime)
{
  Eigen::Isometry3d transform;
  get_trans_with_utime( botframes_, from_frame, to_frame, utime, transform); 

  //To director
  bot_lcmgl_t* lcmgl_fr = bot_lcmgl_init(lcm_->getUnderlyingLCM(), from_frame);
  drawFrame(lcmgl_fr, transform);
}

void App::doRegistration(DP &reference, DP &reading, DP &output, PM::TransformationParameters &T)
{
  // ............do registration.............
  // First ICP loop
  string configName1;
  configName1.append(reg_cfg_.homedir);
  configName1.append("/oh-distro/software/perception/registration/filters_config/icp_trimmed_atlas_finals.yaml");
  registr_->setConfigFile(configName1);
  registr_->getICPTransform(reading, reference);
  PM::TransformationParameters T1 = registr_->getTransform();
  cout << "3D Transformation (Trimmed Outlier Filter):" << endl << T1 << endl;
  DP out1 = registr_->getDataOut();

  // To file, registration advanced %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% EVALUATION
  /*
  // Distance dp_cloud points from KNN in ref
  int line_number = sweep_scans_list_->getNbClouds();
  PM::Matrix distsRead = distancesKNN(reference, reading);
  writeLineToFile(distsRead, "distsBeforeRegistration.txt", line_number);
  // Distance out_trimmed_filter points from KNN in ref
  PM::Matrix distsOut1 = distancesKNN(reference, out1);
  writeLineToFile(distsOut1, "distsAfterSimpleRegistration.txt", line_number);*/

  // To director
  std::stringstream vtk_simpleICP;
  vtk_simpleICP << "accum_simpleICP_";
  vtk_simpleICP << to_string(sweep_scans_list_->getNbClouds());
  vtk_simpleICP << ".vtk";
  bot_lcmgl_t* lcmgl_pc1 = bot_lcmgl_init(lcm_->getUnderlyingLCM(), vtk_simpleICP.str().c_str());
  drawPointCloud(lcmgl_pc1, out1);
  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  // Second ICP loop
  // DP out1 = registr_->getDataOut();
  string configName2;
  configName2.append(reg_cfg_.homedir);
  configName2.append("/oh-distro/software/perception/registration/filters_config/icp_max_atlas_finals.yaml");
  registr_->setConfigFile(configName2);

  registr_->getICPTransform(out1, reference);
  PM::TransformationParameters T2 = registr_->getTransform();
  cout << "3D Transformation (Max Distance Outlier Filter):" << endl << T2 << endl;
  output = registr_->getDataOut();

  // To file, registration advanced %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% EVALUATION
  /*
  // Distance out_max_filter points from KNN in ref
  PM::Matrix distsOut2 = distancesKNN(reference, output);
  writeLineToFile(distsOut2, "distsAfterAdvancedRegistration.txt", line_number);*/
  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  T = T2 * T1;
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
      vector<LidarScan> first_sweep_scans_list_ = scans_queue.front();
      scans_queue.pop_front();

      // To file and drawing
      std::stringstream vtk_fname;
      vtk_fname << "accum_advICP_";
      vtk_fname << to_string(sweep_scans_list_->getNbClouds());
        
      if(sweep_scans_list_->isEmpty())
      {
        current_sweep->populateSweepScan(first_sweep_scans_list_, dp_cloud, sweep_scans_list_->getNbClouds());
        sweep_scans_list_->initializeCollection(*current_sweep);

        // To director (NOTE: Visualization using lcmgl shows (sometimes) imperfections
        // in clouds (random lines). The clouds are actually correct, but lcm cannot manage 
        // so many points and introduces mistakes in visualization.
        bot_lcmgl_t* lcmgl_pc = bot_lcmgl_init(lcm_->getUnderlyingLCM(), "ref_cloud");
        drawPointCloud(lcmgl_pc, dp_cloud);
      }
      else
      {
        TimingUtils::tic();

        DP ref = sweep_scans_list_->getReference().getCloud();
        DP out;
        PM::TransformationParameters Ttot;

        this->doRegistration(ref, dp_cloud, out, Ttot);

        TimingUtils::toc();

        current_sweep->populateSweepScan(first_sweep_scans_list_, out, sweep_scans_list_->getNbClouds());

        // Publish corrected pose estimate:
        // 1. copy robot state
        Eigen::Isometry3d world_to_body_last;
        {
          std::unique_lock<std::mutex> lock(robot_state_mutex_);
          world_to_body_last = world_to_body_msg_.pose;
        } 
        // 2. compute corrected estimate
        Eigen::Isometry3d correctedPose;
        Eigen::Isometry3d correction;
        correction = getTransfParamAsIsometry3d(Ttot);
        correctedPose = world_to_body_last * correction.inverse();
        // 3. publish
        bot_core::pose_t msg_out = getIsometry3dAsBotPose(correctedPose, current_sweep->getUtimeEnd());
        lcm_->publish("POSE_BODY_CORRECTED",&msg_out);

        // Ttot is the full transform to move the input on the reference cloud
        // Store current sweep 
        sweep_scans_list_->addSweep(*current_sweep, Ttot);

        // To director
        bot_lcmgl_t* lcmgl_pc = bot_lcmgl_init(lcm_->getUnderlyingLCM(), vtk_fname.str().c_str());
        //drawPointCloud(lcmgl_pc, out);  
      }
     
      // To file
      vtk_fname << ".vtk";
      savePointCloudVTK(vtk_fname.str().c_str(), sweep_scans_list_->getCurrentCloud().getCloud());
        
      current_sweep->~SweepScan(); 
    }
  }
}

void App::planarLidarHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const bot_core::planar_lidar_t* msg){
    
  if (!pose_initialized_){
    cout << "Estimate not initialized, exiting planarLidarHandler.\n";
    return;
  }
  plotBotFrame("head", "local", msg->utime);
    
  // 1. copy robot state
  Eigen::Isometry3d world_to_body_last;
  {
    std::unique_lock<std::mutex> lock(robot_state_mutex_);
    world_to_body_last = world_to_body_msg_.pose;
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
                                          msg->ranges,msg->intensities,world_to_head_now_,head_to_lidar_);

  // DEBUG: Storage in full sweep structure......%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  // Accumulate current scan in point cloud (projection to default reference "local")
  // only if robot is not walking or stepping
  int robot_behavior_now;
  {
    std::unique_lock<std::mutex> lock(robot_behavior_mutex_);
    robot_behavior_now = robot_behavior_now_;
  }

  if (cl_cfg_.robot_name == "Valkyrie" && robot_behavior_now != 4 && accumulate_
      || cl_cfg_.robot_name == "Atlas" && robot_behavior_now != 2 && accumulate_) 
  // only when transition from walking to standing happens 
  {
    if ( accu_->getCounter() % 240 == 0 )
      cout << accu_->getCounter() << " of " << ca_cfg_.batch_size << " scans collected." << endl;
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

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
    cloud = accu_->getCloud();
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

// Valkyrie
void App::behaviorCallbackValkyrie(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::behavior_t* msg)
{
  std::unique_lock<std::mutex> lock(robot_behavior_mutex_);
  {
    robot_behavior_previous_ = robot_behavior_now_;
    robot_behavior_now_ = msg->behavior;

    if (robot_behavior_now_ != 4 && robot_behavior_previous_ == 4)    
      accumulate_ = TRUE;
  }
}
// Atlas
void App::behaviorCallbackAtlas(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::controller_status_t* msg)
{
  std::unique_lock<std::mutex> lock(robot_behavior_mutex_);
  {
    robot_behavior_previous_ = robot_behavior_now_;
    robot_behavior_now_ = msg->state;

    if (robot_behavior_now_ != 2 && robot_behavior_previous_ == 2)    
      accumulate_ = TRUE;
  }
}

void App::rigidTransformInitHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::rigid_transform_t* msg){
  initState(msg->trans, msg->quat, msg->utime);
}

void App::poseInitHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg){
  initState(msg->pos, msg->orientation, msg->utime);
}

void App::initState(const double trans[3], const double quat[4], long long int t){
  if ( !cl_cfg_.init_with_message ){
    return;
  }
  if ( !pose_initialized_ ){
    cout << "Initialize state estimate using rigid transform or pose.\n";
  }

  std::unique_lock<std::mutex> lock(robot_state_mutex_);
  {
    // Update world to body transform (from kinematics msg) 
    world_to_body_msg_ = getPoseAsIsometryTime3d(trans, quat, t);

    // To director
    bot_lcmgl_t* lcmgl_fr = bot_lcmgl_init(lcm_->getUnderlyingLCM(), "pelvis");
    drawFrame(lcmgl_fr, world_to_body_msg_.pose);

    pose_initialized_ = TRUE;
  }
}

int main(int argc, char **argv){
  CommandLineConfig cl_cfg;
  cl_cfg.robot_name = "Valkyrie";
  cl_cfg.output_channel = "POSE_BODY"; // CREATE NEW CHANNEL... ("POSE_BODY_ICP")

  CloudAccumulateConfig ca_cfg;
  ca_cfg.batch_size = 250; // 240 is about 1 sweep
  ca_cfg.min_range = 0.50; //1.85; // remove all the short range points
  ca_cfg.max_range = 15.0; // we can set up to 30 meters (guaranteed range)
  ca_cfg.lidar_channel ="SCAN";

  RegistrationConfig reg_cfg;
  // Load initial transform
  reg_cfg.initTrans_.clear();
  reg_cfg.initTrans_.append("0,0,0"); 

  ConciseArgs parser(argc, argv, "simple-fusion");
  parser.add(cl_cfg.robot_name, "r", "robot_name", "Atlas or Valkyrie?");
  parser.add(cl_cfg.output_channel, "o", "output_channel", "Output message e.g POSE_BODY");
  parser.add(ca_cfg.lidar_channel, "l", "lidar_channel", "Input message e.g SCAN");
  parser.add(ca_cfg.batch_size, "b", "batch_size", "Number of scans accumulated per 3D point cloud");
  parser.add(ca_cfg.min_range, "m", "min_range", "Closest accepted lidar range");
  parser.add(ca_cfg.max_range, "M", "max_range", "Furthest accepted lidar range");
  parser.parse();

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  App* app= new App(lcm, cl_cfg, ca_cfg, reg_cfg);

  // To director
  bot_lcmgl_t* lcmgl_fr = bot_lcmgl_init(lcm->getUnderlyingLCM(), "local");
  drawFrame(lcmgl_fr);

  while(0 == lcm->handle());
}
