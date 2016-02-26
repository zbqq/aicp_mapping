// sequential-registration (while playing log file)

#include <zlib.h>
#include <lcm/lcm-cpp.hpp>

#include <lcmtypes/bot_core.hpp>
#include <boost/shared_ptr.hpp>
#include <ConciseArgs>

#include <bot_param/param_client.h>
#include <bot_frames/bot_frames.h>

#include <pcl/io/vtk_io.h>

#include <icp-registration/cloud_accumulate.hpp>
#include <icp-registration/icp_3Dreg_and_plot.hpp>
#include <icp-registration/icp_utils.h>
#include <icp-registration/vtkUtils.h>

#include "drawingUtils/drawingUtils.hpp"
#include "aligned_sweeps_collection.hpp"

using namespace std;

struct CommandLineConfig
{
  bool init_with_message; // initialize off of a pose or vicon
  bool vicon_available;
  std::string output_channel;
};

class App{
  public:
    App(boost::shared_ptr<lcm::LCM> &lcm_, const CommandLineConfig& cl_cfg_, 
        CloudAccumulateConfig ca_cfg_, RegistrationConfig reg_cfg_);
    
    ~App(){
    }
    
    CloudAccumulateConfig ca_cfg_;
    RegistrationConfig reg_cfg_;

    void planarLidarHandler(const lcm::ReceiveBuffer* rbuf, 
                            const std::string& channel, const  bot_core::planar_lidar_t* msg);
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    const CommandLineConfig cl_cfg_;
  
    CloudAccumulate* accu_; 
    Registration* registr_;

    BotParam* botparam_;
    BotFrames* botframes_;
    FrameCheckTools frame_check_tools_;

    Laser_projector* laser_projector_;
    laser_projected_scan* projected_laser_scan_;

    // Data structures for storage
    vector<LidarScan> lidar_scans_list_;
    AlignedSweepsCollection* sweep_scans_list_;

    // Transformation matrices
    Eigen::Isometry3d body_to_lidar_; // Variable tf from the lidar to the robot's base link
    Eigen::Isometry3d head_to_lidar_; // Fixed tf from the lidar to the robot's head frame
    Eigen::Isometry3d world_to_body_msg_; // Captures the position of the body frame in world from launch 
                                          // without drift correction
    Eigen::Isometry3d world_to_body_now_; // running position estimate
    Eigen::Isometry3d world_to_head_now_; // running head position estimate

    bool pose_initialized_;

    // MOVED TO PUBLIC (SEE lidar_accumulate.cpp), TRYING TO FIX ERRORS IN ACCUMULATION
    // ALSO NEW CLOUDACCUMULATE MOVED FROM APP CONSTRUCTOR TO MAIN...
    // Init handlers:
    //void planarLidarHandler(const lcm::ReceiveBuffer* rbuf, 
      //                const std::string& channel, const  bot_core::planar_lidar_t* msg);

    void rigidTransformInitHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::rigid_transform_t* msg);
    void poseInitHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg);
    void initState( const double trans[3], const double quat[4]);

    bot_core::pose_t getPoseAsBotPose(Eigen::Isometry3d pose, int64_t utime);
    int get_trans_with_utime(BotFrames *bot_frames,
        const char *from_frame, const char *to_frame, int64_t utime,
        Eigen::Isometry3d& mat);
};    

App::App(boost::shared_ptr<lcm::LCM> &lcm_, const CommandLineConfig& cl_cfg_, 
         CloudAccumulateConfig ca_cfg_, RegistrationConfig reg_cfg_) : lcm_(lcm_), 
         cl_cfg_(cl_cfg_),
         ca_cfg_(ca_cfg_),
         reg_cfg_(reg_cfg_){

  // Storage
  sweep_scans_list_ = new AlignedSweepsCollection();

  // Set up frames and config:
  botparam_ = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 0); // 1 means keep updated, 0 would ignore updates
  botframes_ = bot_frames_get_global(lcm_->getUnderlyingLCM(), botparam_);

  accu_ = new CloudAccumulate(lcm_, ca_cfg_, botparam_, botframes_);

  // Laser projector
  laser_projector_ = accu_->getLaserProjector();
  
  //lcm_->subscribe(ca_cfg_.lidar_channel, &App::planarLidarHandler, this);  
  // bot_frames_structure,from_frame,to_frame,utime,result
  /*int body_status = get_trans_with_utime( botframes_, (ca_cfg_.lidar_channel).c_str(), "body"  , 0, body_to_lidar_);
  int head_status = get_trans_with_utime( botframes_, (ca_cfg_.lidar_channel).c_str(), "head"  , 0, head_to_lidar_);
  if (body_status != 1 || head_status != 1)
    cerr << "Compute rigid body Transformation: FAILED." << endl;*/

  pose_initialized_ = FALSE;
  if (!cl_cfg_.init_with_message){
    std::cout << "Initialize estimate using default.\n";
    world_to_body_msg_ = Eigen::Isometry3d::Identity();

    pose_initialized_ = TRUE;
  }
  else{
    if(cl_cfg_.vicon_available)
    {
      lcm_->subscribe("VICON_BODY|VICON_FRONTPLATE",&App::rigidTransformInitHandler,this);
      lcm_->subscribe("POSE_VICON",&App::poseInitHandler,this);
      std::cout << "Initialization of LIDAR pose using VICON...\n";
    }
    else
    {
      lcm_->subscribe("POSE_BODY",&App::poseInitHandler,this); 
      std::cout << "Initialization of LIDAR pose...\n";
    }  
  }

  registr_ = new Registration(lcm_, reg_cfg_);
  //lidarOdom_ = new LidarOdom(lcm_);
}

int App::get_trans_with_utime(BotFrames *bot_frames,
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

bot_core::pose_t App::getPoseAsBotPose(Eigen::Isometry3d pose, int64_t utime){
  bot_core::pose_t pose_msg;
  pose_msg.utime =   utime;
  pose_msg.pos[0] = pose.translation().x();
  pose_msg.pos[1] = pose.translation().y();
  pose_msg.pos[2] = pose.translation().z();
  Eigen::Quaterniond r_x(pose.rotation());
  pose_msg.orientation[0] =  r_x.w();
  pose_msg.orientation[1] =  r_x.x();
  pose_msg.orientation[2] =  r_x.y();
  pose_msg.orientation[3] =  r_x.z();
  return pose_msg;
}

bot_core_planar_lidar_t* convertPlanarLidarCppToC(std::shared_ptr<bot_core::planar_lidar_t> this_msg){

  bot_core_planar_lidar_t * laser_msg_c = new bot_core_planar_lidar_t;
  laser_msg_c->intensities = new float[this_msg->nintensities];
  laser_msg_c->nintensities = this_msg->nintensities;
  memcpy(laser_msg_c->intensities, &this_msg->intensities[0], this_msg->nintensities * sizeof(float));

  laser_msg_c->ranges = new float[this_msg->nranges];
  laser_msg_c->nranges = this_msg->nranges;
  memcpy(laser_msg_c->ranges, &this_msg->ranges[0], this_msg->nranges * sizeof(float));

  laser_msg_c->rad0 = this_msg->rad0;
  laser_msg_c->radstep = this_msg->radstep;
  laser_msg_c->utime = this_msg->utime;  
  return laser_msg_c;
}

void App::planarLidarHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::planar_lidar_t* msg){
    
  if (!pose_initialized_){
    std::cout << "Estimate not initialized, exiting planarLidarHandler.\n";
    return;
  }
    
  // Populate SweepScan with current LidarScan data structure
  // 1. Get lidar pose
  // bot_frames_structure,from_frame,to_frame,utime,result
  int body_status = get_trans_with_utime( botframes_, (ca_cfg_.lidar_channel).c_str(), "body"  , 0, body_to_lidar_);
  int head_status = get_trans_with_utime( botframes_, (ca_cfg_.lidar_channel).c_str(), "head"  , 0, head_to_lidar_);
  if (body_status != 1 || head_status != 1)
    cerr << "Compute lidar current pose: FAILED." << endl;
  // 2. Compute current_world_to_head
  Eigen::Isometry3d world_to_lidar_now = world_to_body_msg_ * body_to_lidar_;
  world_to_head_now_ = world_to_lidar_now * head_to_lidar_.inverse();
  // 3. Get scan projected to head
  std::shared_ptr<bot_core::planar_lidar_t> converted_msg;
  converted_msg = std::shared_ptr<bot_core::planar_lidar_t>(new bot_core::planar_lidar_t(*msg));
  bot_core_planar_lidar_t* laser_msg_c = convertPlanarLidarCppToC(converted_msg);
  projected_laser_scan_ = laser_create_projected_scan_from_planar_lidar_with_interpolation(laser_projector_, laser_msg_c, "local");
  // 4. Store in LidarScan current scan wrt head frame
  vector<float> ranges;
  for (int i = 0; i < projected_laser_scan_->npoints; i++) {
    ranges.push_back(projected_laser_scan_->rawScan->ranges[i]);
  }
  LidarScan* current_scan = new LidarScan(msg->utime,msg->rad0,msg->radstep,
                                           ranges,msg->intensities,world_to_head_now_);
  lidar_scans_list_.push_back(*current_scan);

  // DEBUG %%%%%%%% they should not be the same... where I am wrong??
  /*
  vector<float> v = current_scan->getRanges();
  for (int i = 0; i < v.size(); i++) {
    cout << "i: " << projected_laser_scan_->rawScan->ranges[i] << " j: " << msg->ranges[i] << endl;
  }*/
  // %%%%%%%%%%%%

  delete current_scan;

  ////////////////////////////////////%%%%%%%%%%%%%%%%%%

  // Accumulate current scan in point cloud (projection to default reference "local")
  if ( accu_->getCounter() % 200 == 0)
    cout << accu_->getCounter() << " of " << ca_cfg_.batch_size << " scans collected." << endl;
  accu_->processLidar(converted_msg);
 
  if ( accu_->getFinished() ){ //finished accumulating?

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
    cloud = accu_->getCloud();
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cout << "Processing cloud with " << cloud->points.size() << " points." << endl;

    // For storing current cloud
    DP dp_cloud;
    fromPCLToDataPoints(dp_cloud, *cloud);

    SweepScan* current_sweep = new SweepScan();

    // To file and drawing
    std::stringstream vtk_fname;
    vtk_fname << "accum_cloud_";
    vtk_fname << to_string(sweep_scans_list_->getNbClouds());
    
    if(sweep_scans_list_->isEmpty())
    {
      current_sweep->populateSweepScan(lidar_scans_list_, dp_cloud, sweep_scans_list_->getNbClouds());
      sweep_scans_list_->initializeCollection(*current_sweep);

      //To director
      bot_lcmgl_t* lcmgl_pc = bot_lcmgl_init(lcm_->getUnderlyingLCM(), "Point Cloud");
      drawPointCloud(lcmgl_pc, dp_cloud);
    }
    else
    {
      DP ref = sweep_scans_list_->getReference().getCloud();
      // ............call registration.............
      // First ICP loop
      registr_->getICPTransform(dp_cloud, ref);
      PM::TransformationParameters T1 = registr_->getTransform();
      cout << "3D Transformation (Trimmed Outlier Filter):" << endl << T1 << endl;
      
      // Second ICP loop
      DP out1 = registr_->getDataOut();
      string configName2;
      configName2.append(reg_cfg_.homedir);
      configName2.append("/oh-distro/software/perception/registration/filters_config/icp_max_atlas_finals.yaml");
      registr_->setConfigFile(configName2);

      registr_->getICPTransform(out1, ref);
      PM::TransformationParameters T2 = registr_->getTransform();
      cout << "3D Transformation (Max Distance Outlier Filter):" << endl << T2 << endl;
      DP out2 = registr_->getDataOut();

      PM::TransformationParameters Ttot = T2 * T1;      
      current_sweep->populateSweepScan(lidar_scans_list_, out2, sweep_scans_list_->getNbClouds());
      // Ttot is the full transform to move the input on the reference cloud
      // Project current sweep in head frame
      // Store current sweep 
      sweep_scans_list_->addSweep(*current_sweep, Ttot);

      //To director
      bot_lcmgl_t* lcmgl_pc = bot_lcmgl_init(lcm_->getUnderlyingLCM(), vtk_fname.str().c_str());
      drawPointCloud(lcmgl_pc, out2);      
    }
 
    // To file
    vtk_fname << ".vtk";
    savePointCloudVTK(vtk_fname.str().c_str(), sweep_scans_list_->getCurrentCloud().getCloud());
    
    lidar_scans_list_.clear();
    current_sweep->~SweepScan(); 



    accu_->clearCloud();/*
    while (!frame_check_tools_.isLocalToScanUpdated(botframes_, msg->utime)){
      cout << "Current msg utime: " << msg->utime << endl;
      cout << "Is local to scan chain udated? NO.%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl; 
    }*/
    //delete accu_;
    //accu_ = new CloudAccumulate(lcm_, ca_cfg_, botparam_, botframes_);

    //cin.get();
  }
}

// TO BE UPDATED: POINT CLOUD HANDLER
/*
void App::lidarHandler(const lcm::ReceiveBuffer* rbuf,
     const std::string& channel, const  bot_core::planar_lidar_t* msg){
  if (!pose_initialized_){
    std::cout << "Estimate not initialised, exiting lidarHandler\n";
    return;
  }

  // 1. Update LIDAR Odometry
  std::vector<float> ranges_copy = msg->ranges;
  float* ranges = &ranges_copy[0];
  lidarOdom_->doOdometry(ranges, msg->nranges, msg->rad0, msg->radstep, msg->utime);

  // 2. Determine the body position using the LIDAR motion estimate:
  Eigen::Isometry3d lidar_init_to_lidar_now = lidarOdom_->getCurrentPose();
  Eigen::Isometry3d world_to_lidar_now = world_to_body_init_*body_to_lidar_*lidar_init_to_lidar_now;
  world_to_body_now_ = world_to_lidar_now * body_to_lidar_.inverse();

  //bot_core::pose_t pose_msg = getPoseAsBotPose( lidar_init_to_lidar_now , msg->utime);
  //lcm_->publish("POSE_BODY_ALT", &pose_msg );
  bot_core::pose_t pose_msg_body = getPoseAsBotPose( world_to_body_now_ , msg->utime);
  lcm_->publish(cl_cfg_.output_channel, &pose_msg_body );
}*/

void App::initState(const double trans[3], const double quat[4]){
  if ( !cl_cfg_.init_with_message ){
    return;
  }
  if ( !pose_initialized_ ){
    std::cout << "Initialize state estimate using rigid transform or pose.\n";
  }

  // Update world to body transform (from kinematics msg) 
  world_to_body_msg_.setIdentity();
  world_to_body_msg_.translation()  << trans[0], trans[1] , trans[2];
  Eigen::Quaterniond quatE = Eigen::Quaterniond(quat[0], quat[1], 
                                               quat[2], quat[3]);
  world_to_body_msg_.rotate(quatE);

  pose_initialized_ = TRUE;
}

void App::rigidTransformInitHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::rigid_transform_t* msg){
  initState(msg->trans, msg->quat);
}

void App::poseInitHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg){
  initState(msg->pos, msg->orientation);
}

int main(int argc, char **argv){
  CommandLineConfig cl_cfg;
  cl_cfg.init_with_message = TRUE;
  cl_cfg.vicon_available = FALSE;
  cl_cfg.output_channel = "POSE_BODY"; // CREATE NEW CHANNEL... ("POSE_BODY_ICP")

  CloudAccumulateConfig ca_cfg;
  ca_cfg.batch_size = 240; // about 1 sweep
  ca_cfg.min_range = 0.50; //1.85; // remove all the short range points
  ca_cfg.max_range = 15.0; // we can set up to 30 meters (guaranteed range)
  ca_cfg.lidar_channel ="SCAN";

  RegistrationConfig reg_cfg;
  reg_cfg.configFile3D_.clear();
  // Load first config file (trimmed distance outlier filter)
  reg_cfg.configFile3D_.append(reg_cfg.homedir);
  reg_cfg.configFile3D_.append("/oh-distro/software/perception/registration/filters_config/icp_trimmed_atlas_finals.yaml");
  reg_cfg.initTrans_.clear();
  reg_cfg.initTrans_.append("0,0,0"); 

  ConciseArgs parser(argc, argv, "simple-fusion");
  parser.add(cl_cfg.init_with_message, "g", "init_with_message", "Bootstrap internal estimate using VICON or POSE_BODY");
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
  //App app = App(lcm, cl_cfg, ca_cfg, reg_cfg);
  App* app= new App(lcm, cl_cfg, ca_cfg, reg_cfg);

  lcm->subscribe(ca_cfg.lidar_channel, &App::planarLidarHandler, app);

  while(0 == lcm->handle());
}
