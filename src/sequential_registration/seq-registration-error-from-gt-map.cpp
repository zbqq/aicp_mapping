// Run: error-from-gt-map -h

// Input: POSE_BODY
// Output: ERRORS
// Computes T registering current point cloud against a ground truth (gt)
// point cloud of the entire area (100% overlap). T represents translation 
// and rotation errors at the frequency of registration (~1/6s).

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
  std::string ground_truth_clouds;
  std::string pose_body_channel;
  std::string output_channel;
  std::string working_mode;
  std::string algorithm;
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
    Eigen::Isometry3d world_to_head_now_; // running head position estimate

    bool pose_initialized_;
    bool updated_correction_;

    // Visualisation
    pronto_vis* pc_vis_;

    // Reference point cloud (ground truth from user)
    DP reference_gt_; 
    // Temporary config file for ICP chain: copied and trimmed ratio replaced
    string tmpConfigName_;

    // Correction variables
    Eigen::Isometry3d current_correction_;

    // Residuals params
    float residualMeanDist_;
    float residualMedDist_;
    float residualQuantDist_;

    // Init handlers:
    void planarLidarHandler(const lcm::ReceiveBuffer* rbuf, 
                            const std::string& channel, const  bot_core::planar_lidar_t* msg);
    void doRegistration(DP &reference, DP &reading, DP &output, PM::TransformationParameters &T);

    void poseInitHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg);

    bot_core::pose_t getIsometry3dAsBotPose(Eigen::Isometry3d pose, int64_t utime);
    Eigen::Isometry3d getPoseAsIsometry3d(const bot_core::pose_t* pose);
    Eigen::Isometry3d getBodyAsIsometry3d(const bot_core::rigid_transform_t* pose);
    Eigen::Isometry3d getTransfParamAsIsometry3d(PM::TransformationParameters T);
};

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

void parseString(string cloud_indexes_string, vector<int>& cloud_indexes_list)
{
  cloud_indexes_string.erase(std::remove(cloud_indexes_string.begin(), cloud_indexes_string.end(), '['),
            cloud_indexes_string.end());
  cloud_indexes_string.erase(std::remove(cloud_indexes_string.begin(), cloud_indexes_string.end(), ']'),
            cloud_indexes_string.end());
  std::replace( cloud_indexes_string.begin(), cloud_indexes_string.end(), ',', ' ');
  std::replace( cloud_indexes_string.begin(), cloud_indexes_string.end(), ';', ' ');

  stringstream cloudsStringStream(cloud_indexes_string);
  float tmp;
  while (!cloudsStringStream.eof())
  {
    if (cloudsStringStream.peek() != ' ')
    {
      if(!(cloudsStringStream >> tmp))
        cerr << "An error occured while trying to parse the string." << endl;
      else
        cloud_indexes_list.push_back(tmp);

    }
    else { cloudsStringStream.get(); }
  }
}   

App::App(boost::shared_ptr<lcm::LCM> &lcm_, const CommandLineConfig& cl_cfg_, 
         CloudAccumulateConfig ca_cfg_, RegistrationConfig reg_cfg_) : lcm_(lcm_), 
         cl_cfg_(cl_cfg_), ca_cfg_(ca_cfg_), reg_cfg_(reg_cfg_){

  pose_initialized_ = FALSE;
  updated_correction_ = TRUE;

  // Load clouds to build ground truth model
  if (cl_cfg_.ground_truth_clouds.empty())
    std::cerr <<"ERROR: point clouds missing. Reference cloud cannot be generated." <<std::endl;
  else 
  {
    vector<int> input_cloud_names;

    parseString(cl_cfg_.ground_truth_clouds, input_cloud_names);
    // Load point clouds from file
    DP tmp_cloud;
    for (int i = 0; i < input_cloud_names.size(); i++)
    {
      std::stringstream cloud_name;
      cloud_name << "ref_to_merge_";
      cloud_name << to_string(input_cloud_names.at(i));
      cloud_name << ".vtk";

      if(tmp_cloud.getNbPoints() == 0)
        tmp_cloud = DP::load(cloud_name.str());
      else 
        tmp_cloud.concatenate(DP::load(cloud_name.str()));
    }
    reference_gt_ = tmp_cloud;
    // To file: DEBUG
    std::stringstream vtk_fname;
    vtk_fname << "reference.vtk";
    savePointCloudVTK(vtk_fname.str().c_str(), reference_gt_);
  }

  // File used to update config file for ICP chain
  tmpConfigName_.append(REG_BASE);
  tmpConfigName_.append("/filters_config/icp_autotuned_default.yaml");

  local_ = Eigen::Isometry3d::Identity();
  world_to_body_msg_ = Eigen::Isometry3d::Identity();
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

  // Visualiser
  pc_vis_ = new pronto_vis( lcm_->getUnderlyingLCM() );

  // Pose initialization
  lcm_->subscribe(cl_cfg_.pose_body_channel, &App::poseInitHandler, this);
  cout << "Initialization of robot pose...\n";

  // ICP chain
  registr_ = new Registration(reg_cfg_);
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

void App::doRegistration(DP &reference, DP &reading, DP &output, PM::TransformationParameters &T)
{
  DP non_filtered_reading(reading);

  // PRE-FILTERING using filteringUtils
  if (cl_cfg_.algorithm == "aicp")
  {
    regionGrowingPlaneSegmentationFilter(reference);
    regionGrowingPlaneSegmentationFilter(reading);
  }

  // To director current cloud (Point Cloud 5, Frame 5)
  drawPointCloudCollections(lcm_, 5, local_, reference, 1);
  // To director current cloud (Point Cloud 6, Frame 6)
  drawPointCloudCollections(lcm_, 6, local_, reading, 1);

  // ............do registration.............
  // First ICP loop
  string configName1;
  configName1.append(REG_BASE);
  if (cl_cfg_.algorithm == "icp")
    configName1.append("/filters_config/Chen91_pt2plane.yaml");
  else if (cl_cfg_.algorithm == "aicp")
    configName1.append("/filters_config/icp_autotuned.yaml");

  if (cl_cfg_.algorithm == "aicp")
    replaceRatioConfigFile(tmpConfigName_, configName1, 0.70);

  registr_->setConfigFile(configName1);
  registr_->getICPTransform(reading, reference);
  PM::TransformationParameters T1 = registr_->getTransform();
  cout << "3D Transformation (Trimmed Outlier Filter):" << endl << T1 << endl;
  // Store output after first ICP
  // (if second ICP gives exception the output cloud remains this one)
  output = registr_->getDataOut();

  PM::ICP icp = registr_->getIcp();
  DP readFiltered = icp.getReadingFiltered();

  getResidualError(icp, 0.70, residualMeanDist_, residualMedDist_, residualQuantDist_);

  // To director aligned reading cloud (Point Cloud 1, Frame 1)
  drawPointCloudCollections(lcm_, 1, local_, output, 1);

  //Original reading aligned
  icp.transformations.apply(non_filtered_reading, T1);
  // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  /*// To file: DEBUG
  std::stringstream vtk_fname;
  vtk_fname << "reading_";
  vtk_fname << to_string(sweep_scans_list_->getNbClouds());
  vtk_fname << ".vtk";
  savePointCloudVTK(vtk_fname.str().c_str(), not_filtered_reading);*/
  // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  T = T1;
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
      vector<LidarScan> planar_scans_list = scans_queue.front();
      scans_queue.pop_front();

      if(sweep_scans_list_->isEmpty())
      {
        vector<LidarScan> empty_sweep_scans_list;

        LidarScan* empty_scan = new LidarScan(0,0,0,
                                {},{},Eigen::Isometry3d::Identity(),Eigen::Isometry3d::Identity(),
                                Eigen::Isometry3d::Identity());

        empty_sweep_scans_list.push_back(*empty_scan);
        current_sweep->populateSweepScan(empty_sweep_scans_list, reference_gt_, sweep_scans_list_->getNbClouds());
        sweep_scans_list_->initializeCollection(*current_sweep);
      }

      // Get reference
      DP ref = sweep_scans_list_->getCurrentReference().getCloud();

      // To director first reference cloud (Point Cloud 0, Frame 0)
      drawPointCloudCollections(lcm_, 0, local_, reference_gt_, 1);

      TimingUtils::tic();

      DP out;
      PM::TransformationParameters Ttot;

      this->doRegistration(ref, dp_cloud, out, Ttot);

      TimingUtils::toc();

      // Compute correction to pose estimate:
      current_sweep->populateSweepScan(planar_scans_list, out, sweep_scans_list_->getNbClouds(), sweep_scans_list_->getCurrentReference().getId(), TRUE);

      current_correction_ = getTransfParamAsIsometry3d(Ttot);
      updated_correction_ = TRUE;

      // Publish
      bot_core::double_array_t msg_out;

      // Apply correction if available (identity otherwise)
      if (updated_correction_)
      {
        updated_correction_ = FALSE;

        if (residualMedDist_ < 0.003 && residualQuantDist_ < 0.004)
        {
          /*// To correct robot drift publish COMPUTED_ERROR_TRANSFORM
          msg_out = getIsometry3dAsBotPose(current_correction_, current_sweep->getUtimeEnd());
          lcm_->publish(cl_cfg_.output_channel,&msg_out);*/

          Eigen::Isometry3d world_to_body_end_of_sweep = current_sweep->getBodyPose();
          Eigen::Isometry3d corrected_pose = current_correction_ * world_to_body_end_of_sweep;

          // Publish ESTIMATED_POSE and CORRECTED_POSE to Director
          drawFrameCollections(lcm_, 7, world_to_body_end_of_sweep, current_sweep->getUtimeEnd());
          drawFrameCollections(lcm_, 8, corrected_pose, current_sweep->getUtimeEnd());

          // Compute difference between estimated and corrected
          bot_core::pose_t est_pose = getIsometry3dAsBotPose(world_to_body_end_of_sweep, current_sweep->getUtimeEnd());
          bot_core::pose_t corr_pose = getIsometry3dAsBotPose(corrected_pose, current_sweep->getUtimeEnd());
          // Translation
          float transl_err = sqrt( pow(corr_pose.pos[0] - est_pose.pos[0], 2.0) + pow(corr_pose.pos[1] - est_pose.pos[1], 2.0) + pow(corr_pose.pos[2] - est_pose.pos[2], 2.0));
          // Rotation
          Eigen::Matrix3d delta_rot = world_to_body_end_of_sweep.rotation() * corrected_pose.rotation().inverse();
          float trace_rot = delta_rot(0,0) + delta_rot(1,1) + delta_rot(2,2);
          float rot_err = acos ( (trace_rot-1)/2.0 ) * 180.0 / M_PI;

          // To correct robot drift publish POSE_ERRORS
          msg_out.utime = current_sweep->getUtimeEnd();
          msg_out.num_values = 2;
          msg_out.values.push_back(transl_err);
          msg_out.values.push_back(rot_err);

          lcm_->publish(cl_cfg_.output_channel,&msg_out);
        }
        else
        {
          cout << "Alignment not trusted: Correction DISCARDED." << endl;
        }
      }

      // Ttot is the full transform to move the input on the reference cloud
      // Store current sweep 
      sweep_scans_list_->addSweep(*current_sweep, Ttot);

      // DEBUG
      cout << "REFERENCE: " << sweep_scans_list_->getCurrentReference().getId() << endl;
      cout << "Cloud ID: " << sweep_scans_list_->getCurrentCloud().getId() << endl;
      cout << "Number Clouds: " << sweep_scans_list_->getNbClouds() << endl;

      // To file
      /*
      std::stringstream vtk_fname;
      vtk_fname << "cloud_";
      vtk_fname << to_string(sweep_scans_list_->getNbClouds()-1);
      vtk_fname << ".vtk";
      savePointCloudVTK(vtk_fname.str().c_str(), sweep_scans_list_->getCurrentCloud().getCloud());*/
        
      current_sweep->~SweepScan();

      cout << "--------------------------------------------------------------------------------------" << endl;
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

  if ( accu_->getCounter() % ca_cfg_.batch_size == 0 ) {
    cout << accu_->getCounter() << " of " << ca_cfg_.batch_size << " scans collected." << endl;
  }
  accu_->processLidar(msg);
  lidar_scans_list_.push_back(*current_scan);

  delete current_scan;

  if ( accu_->getFinished() ){ //finished accumulating?
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

void App::poseInitHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg){

  Eigen::Isometry3d world_to_body_last;
  std::unique_lock<std::mutex> lock(robot_state_mutex_);
  {
    // Update world to body transform (from kinematics msg) 
    world_to_body_msg_ = getPoseAsIsometry3d(msg);
    world_to_body_last = world_to_body_msg_;
  }

  pose_initialized_ = TRUE;
}


int main(int argc, char **argv){
  CommandLineConfig cl_cfg;
  cl_cfg.ground_truth_clouds.clear();
  cl_cfg.algorithm = "aicp";
  cl_cfg.pose_body_channel = "POSE_BODY";
  //cl_cfg.output_channel = "COMPUTED_ERROR_TRANSFORM"; // Create new channel...
  cl_cfg.output_channel = "POSE_ERRORS"; // Create new channel...

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
  parser.add(cl_cfg.ground_truth_clouds, "gt", "ground_truth_clouds", "Clouds to merge to generate ground truth reference (e.g [1,2,3])");
  parser.add(cl_cfg.algorithm, "a", "algorithm", "AICP or ICP? (i.e. aicp or icp)");
  parser.add(cl_cfg.pose_body_channel, "p", "pose_body_channel", "Kinematics-inertia pose estimate");
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
