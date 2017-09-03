// Run: aicp-registration-online -h
// Testing: aicp-registration-online -s debug -b 83 -a -v

// Input: POSE_BODY
// Output: POSE_BODY_CORRECTED
// Computes T_AICP and corrects the state estimate in POSE_BODY message.

// The algorithm accumulates scans on a thread and registers the accumulated scans (clouds) in parallel
// on a second thread. The first cloud is selected as the first reference for registration. The reference
// cloud is updated with last aligned cloud
//     if (alignment_risk > threshold).

#include <sstream>  // stringstream
#include <map>
#include <random>

// Project lib
#include "aicpRegistration/registration.hpp"
#include "aicpRegistration/common.hpp"
#include "aicpRegistration/aligned_sweeps_collection.hpp"

#include "aicpOverlap/overlap.hpp"
#include "aicpOverlap/common.hpp"

#include "aicpClassification/classification.hpp"
#include "aicpClassification/common.hpp"

#include "commonUtils/cloudIO.h"
#include "commonUtils/timing.hpp"
#include "commonUtils/common.hpp"
#include "drawingUtils/drawingUtils.hpp"

// accumulator
#include <cloud_accumulate/cloud_accumulate.hpp>

// yaml
#include "yaml-cpp/yaml.h" // read the yaml config

// lcm
#include <lcm/lcm-cpp.hpp>
#include <boost/shared_ptr.hpp>
//#include <boost/assign/std/vector.hpp>
//#include <boost/filesystem.hpp>

#include <lcmtypes/octomap_utils.h>
#include <lcmtypes/bot_core/planar_lidar_t.hpp>
#include <lcmtypes/bot_core/pose_t.hpp>
#include <lcmtypes/bot_core/rigid_transform_t.hpp>
#include <lcmtypes/bot_core/double_array_t.hpp>

//#include <zlib.h>

// thread
#include <mutex>
#include <condition_variable>
#include <thread>

// args
#include <ConciseArgs>

// tf
#include <bot_param/param_client.h>
#include <bot_frames/bot_frames.h>

// pcl
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
//#include <pcl/io/vtk_io.h>

using namespace std;
using namespace aicp;

struct CommandLineConfig
{
  string configFile;
  string pose_body_channel;
//  string vicon_channel;
  string output_channel;
  string working_mode;
  bool verbose;
//  bool register_if_walking;
  bool apply_correction;
};

class App{
  public:
    App(boost::shared_ptr<lcm::LCM> &lcm_, const CommandLineConfig& cl_cfg_, 
        CloudAccumulateConfig ca_cfg_,
        RegistrationParams reg_params_, OverlapParams overlap_params_,
        ClassificationParams class_params_, string exp_params_);
    
    ~App(){
    }
    
    CloudAccumulateConfig ca_cfg_;
    RegistrationParams reg_params_;
    OverlapParams overlap_params_;
    ClassificationParams class_params_;
    string exp_params_;

    // thread function for doing actual work
    void operator()();
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    const CommandLineConfig cl_cfg_;
  
    CloudAccumulate* accu_; 
    std::unique_ptr<AbstractRegistrator> registr_;
    std::unique_ptr<AbstractOverlapper> overlapper_;
    std::unique_ptr<AbstractClassification> classifier_;

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
    std::mutex cloud_accumulate_mutex_;
    // %%%%%%%%%%%%%%%%%%%%%%%%

    BotParam* botparam_;
    BotFrames* botframes_;

    // Data structures for storage
    vector<LidarScan> lidar_scans_list_;
    AlignedSweepsCollection* sweep_scans_list_;

    Eigen::Matrix4f initialT_;

    // Transformation matrices
    Eigen::Isometry3d local_;
    Eigen::Isometry3d body_to_lidar_; // Variable tf from the lidar to the robot's base link
    Eigen::Isometry3d head_to_lidar_; // Fixed tf from the lidar to the robot's head frame
    Eigen::Isometry3d world_to_body_msg_; // Captures the position of the body frame in world from launch
                                          // without drift correction
    long long int world_to_body_msg_utime_;
    Eigen::Isometry3d world_to_head_now_; // running head position estimate
    Eigen::Isometry3d world_to_body_corr_first_;

    bool pose_initialized_;
    bool updated_correction_;

    // Avoid cloud distortion clearing buffer after pose update
    bool clear_clouds_buffer_;

    // Create debug data folder
    std::stringstream data_directory_path_;

    // Overlap parameters
    float fov_overlap_;
    float octree_overlap_;
    // Alignability
    float alignability_;
    // Alignment Risk
    Eigen::MatrixXd risk_prediction_; // ours

    // Correction variables
    bool valid_correction_;
    bool force_reference_update_;
    Eigen::Isometry3d corrected_pose_;
    Eigen::Isometry3d current_correction_;

    // Reference cloud update counters
    int updates_counter_;

    // Used for: convertCloudProntoToPcl
    pronto_vis* pc_vis_;

    // Write to file
    pcl::PCDWriter pcd_writer_;

    // Init handlers:
    void planarLidarHandler(const lcm::ReceiveBuffer* rbuf,
                            const std::string& channel, const  bot_core::planar_lidar_t* msg);
    void doRegistration(pcl::PointCloud<pcl::PointXYZ>& reference, pcl::PointCloud<pcl::PointXYZ>& reading,
                        Eigen::Matrix4f &T, vector<float>& failure_prediction_factors);

    void poseInitHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg);

    // Tool functions
    bot_core::pose_t getIsometry3dAsBotPose(Eigen::Isometry3d pose, int64_t utime);
    Eigen::Isometry3d getPoseAsIsometry3d(const bot_core::pose_t* pose);
    Eigen::Isometry3d getBodyAsIsometry3d(const bot_core::rigid_transform_t* pose);
    Eigen::Isometry3d getTransfParamAsIsometry3d(PM::TransformationParameters T);
};    

App::App(boost::shared_ptr<lcm::LCM> &lcm_, const CommandLineConfig& cl_cfg_, 
         CloudAccumulateConfig ca_cfg_,
         RegistrationParams reg_params_, OverlapParams overlap_params_,
         ClassificationParams class_params_, string exp_params_) :
         lcm_(lcm_), cl_cfg_(cl_cfg_), ca_cfg_(ca_cfg_),
         reg_params_(reg_params_), overlap_params_(overlap_params_),
         class_params_(class_params_), exp_params_(exp_params_){

  pose_initialized_ = FALSE;
  updated_correction_ = TRUE;

  valid_correction_ = FALSE;
  force_reference_update_ = FALSE;

  // Reference cloud update counters
  updates_counter_ = 0;

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
  cout << "[Main] Degeneracy (degenerate if ~ 0): " << failure_prediction_factors.at(0) << " %" << endl;
  cout << "[Main] ICN (degenerate if ~ 0): " << failure_prediction_factors.at(1) << endl;

  cout << "==============================" << endl
       << "[Main] Computed 3D Transform:" << endl
       << "==============================" << endl
       << T << endl;

//  T =  T * initialT_;

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
        vector<float> other_predictions;

        this->doRegistration(*ref_xyz_prefiltered, *read_xyz_prefiltered,
                             Ttot, other_predictions);

        if(risk_prediction_(0,0) > class_params_.svm.threshold)
        {
            cout << "====================================" << endl
                 << "[Main] REFERENCE UPDATE" << endl
                 << "====================================" << endl;
          // Reference Update Statistics
          updates_counter_ ++;

          // Updating reference with current reading (non-aligned)
          current_sweep->populateSweepScan(first_sweep_scans_list, *read_ptr, sweep_scans_list_->getNbClouds(), -1, 1);
          sweep_scans_list_->addSweep(*current_sweep, initialT_);

          int current_cloud_id = sweep_scans_list_->getCurrentCloud().getId();
          bool referenceSet = sweep_scans_list_->getCloud(current_cloud_id).setReference();
          cout << "NEW REFERENCE is " << current_cloud_id << ", Set: " << referenceSet << endl;
          if (referenceSet)
            sweep_scans_list_->updateReference(current_cloud_id);

          // To file
          stringstream ss_tmp3;
          ss_tmp3 << data_directory_path_.str();
          ss_tmp3 << "/ref_";
          ss_tmp3 << to_string(sweep_scans_list_->getCurrentReference().getId());
          ss_tmp3 << ".pcd";
          pcd_writer_.write<pcl::PointXYZRGB> (ss_tmp3.str (), *sweep_scans_list_->getCurrentReference().getCloud(), false);
        }
        else
        {
          bool enableRef = TRUE;
          pcl::PointCloud<pcl::PointXYZRGB>::Ptr output (new pcl::PointCloud<pcl::PointXYZRGB>);
          pcl::transformPointCloud (*read_ptr, *output, Ttot);
          current_sweep->populateSweepScan(first_sweep_scans_list, *output, sweep_scans_list_->getNbClouds(), sweep_scans_list_->getCurrentReference().getId(), enableRef);
          sweep_scans_list_->addSweep(*current_sweep, Ttot);

          current_correction_ = getTransfParamAsIsometry3d(Ttot); // Ttot transforms initialized reading into reference
          updated_correction_ = TRUE;

          initialT_ = Ttot * initialT_;
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
  long long int world_to_body_last_utime;
  std::unique_lock<std::mutex> lock(robot_state_mutex_);
  {
    // Update world to body transform (from kinematics msg)
    world_to_body_msg_ = getPoseAsIsometry3d(msg);
    world_to_body_msg_utime_ = msg->utime;
    world_to_body_last = world_to_body_msg_;
    world_to_body_last_utime = world_to_body_msg_utime_;
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

    if (updated_correction_)
    {
      {
        std::unique_lock<std::mutex> lock(cloud_accumulate_mutex_);
        clear_clouds_buffer_ = TRUE;
        updated_correction_ = FALSE;
      }
    }
  }

  // Publish current overlap
  bot_core::double_array_t msg_overlap;
  msg_overlap.utime = msg->utime;
  msg_overlap.num_values = 1;
  msg_overlap.values.push_back(octree_overlap_);
  lcm_->publish("OVERLAP",&msg_overlap);

  if ( !pose_initialized_ ){
    world_to_body_corr_first_ = corrected_pose_;
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


int main(int argc, char **argv){
  CommandLineConfig cl_cfg;
  cl_cfg.configFile.append(CONFIG_LOC);
  cl_cfg.configFile.append(PATH_SEPARATOR);
  cl_cfg.configFile.append("aicp_config.yaml");
  cl_cfg.working_mode = "robot";
  cl_cfg.verbose = FALSE;
//  cl_cfg.register_if_walking = TRUE;
  cl_cfg.apply_correction = FALSE;
  cl_cfg.pose_body_channel = "POSE_BODY";
//  cl_cfg.vicon_channel = "VICON_pelvis_val";
  cl_cfg.output_channel = "POSE_BODY_CORRECTED"; // Create new channel...

  CloudAccumulateConfig ca_cfg;
  ca_cfg.batch_size = 240; // 240 is about 1 sweep
  ca_cfg.min_range = 0.50; //1.85; // remove all the short range points
  ca_cfg.max_range = 15.0; // we can set up to 30 meters (guaranteed range)
  ca_cfg.lidar_channel ="MULTISENSE_SCAN";
  //ca_cfg.check_local_to_scan_valid = FALSE;

  ConciseArgs parser(argc, argv, "aicp-registration-online");
  parser.add(cl_cfg.configFile, "c", "config_file", "Config file location");
  parser.add(cl_cfg.working_mode, "s", "working_mode", "Robot or Debug? (i.e. robot or debug)"); //Debug if I want to visualize moving frames in Director
  parser.add(cl_cfg.verbose, "v", "verbose", "Publish frames and clouds to LCM for debug");
  parser.add(cl_cfg.apply_correction, "a", "apply_correction", "Initialize ICP with corrected pose? (during debug)");
  parser.add(cl_cfg.pose_body_channel, "pc", "pose_body_channel", "Kinematics-inertia pose estimate");
//  parser.add(cl_cfg.vicon_channel, "vc", "vicon_channel", "Ground truth pose from Vicon");
  parser.add(cl_cfg.output_channel, "oc", "output_channel", "Corrected pose");
  parser.add(ca_cfg.lidar_channel, "l", "lidar_channel", "Input message e.g MULTISENSE_SCAN");
  parser.add(ca_cfg.batch_size, "b", "batch_size", "Number of scans per full 3D point cloud (at 5RPM)");
  parser.add(ca_cfg.min_range, "m", "min_range", "Closest accepted lidar range");
  parser.add(ca_cfg.max_range, "M", "max_range", "Furthest accepted lidar range");
  parser.parse();

  RegistrationParams registration_params;
  OverlapParams overlap_params;
  ClassificationParams classification_params;
  string experiments_param;
  /*===================================
  =            YAML Config            =
  ===================================*/
  string yamlConfig_;
  YAML::Node yn_;
  yamlConfig_ = cl_cfg.configFile;
  yn_ = YAML::LoadFile(yamlConfig_);

  YAML::Node registrationNode = yn_["AICP"]["Registration"];
  for(YAML::const_iterator it=registrationNode.begin();it != registrationNode.end();++it) {

    const string key = it->first.as<string>();

    if(key.compare("type") == 0) {
      registration_params.type = it->second.as<string>();
    }
    else if(key.compare("sensorRange") == 0) {
      registration_params.sensorRange =  it->second.as<float>();
    }
    else if(key.compare("sensorAngularView") == 0) {
      registration_params.sensorAngularView =  it->second.as<float>();
    }
    else if(key.compare("loadPosesFrom") == 0) {
      registration_params.loadPosesFrom = it->second.as<string>();
    }
    else if(key.compare("initialTransform") == 0) {
      registration_params.initialTransform = it->second.as<string>();
    }
    else if(key.compare("saveCorrectedPose") == 0) {
      registration_params.saveCorrectedPose =  it->second.as<bool>();
    }
    else if(key.compare("saveInitializedReadingCloud") == 0) {
      registration_params.saveInitializedReadingCloud =  it->second.as<bool>();
    }
    else if(key.compare("saveRegisteredReadingCloud") == 0) {
      registration_params.saveRegisteredReadingCloud =  it->second.as<bool>();
    }
    else if(key.compare("enableLcmVisualization") == 0) {
      registration_params.enableLcmVisualization =  it->second.as<bool>();
    }
  }
  if(registration_params.type.compare("Pointmatcher") == 0) {

    YAML::Node pointmatcherNode = registrationNode["Pointmatcher"];

    for(YAML::const_iterator it=pointmatcherNode.begin();it != pointmatcherNode.end();++it) {
      const string key = it->first.as<string>();

      if(key.compare("configFileName") == 0) {
        registration_params.pointmatcher.configFileName.append(FILTERS_CONFIG_LOC);
        registration_params.pointmatcher.configFileName.append(PATH_SEPARATOR);
        registration_params.pointmatcher.configFileName = FILTERS_CONFIG_LOC + PATH_SEPARATOR + it->second.as<string>();
      }
      else if(key.compare("printOutputStatistics") == 0) {
        registration_params.pointmatcher.printOutputStatistics =  it->second.as<bool>();
      }
    }
  }
  else if(registration_params.type.compare("GICP") == 0) {

    YAML::Node gicpNode = registrationNode["GICP"];

    for(YAML::const_iterator it=gicpNode.begin();it != gicpNode.end();++it) {
      const string key = it->first.as<string>();
      const float val = it->second.as<float>();
    }
  }
  YAML::Node overlapNode = yn_["AICP"]["Overlap"];
  for(YAML::const_iterator it=overlapNode.begin();it != overlapNode.end();++it) {

    const string key = it->first.as<string>();

    if(key.compare("type") == 0) {
      overlap_params.type = it->second.as<string>();
    }
  }
  if(overlap_params.type.compare("OctreeBased") == 0) {

    YAML::Node octreeBasedNode = overlapNode["OctreeBased"];

    for(YAML::const_iterator it=octreeBasedNode.begin();it != octreeBasedNode.end();++it) {
      const string key = it->first.as<string>();

      if(key.compare("octomapResolution") == 0) {
        overlap_params.octree_based.octomapResolution = it->second.as<float>();
      }
    }
  }
  YAML::Node classificationNode = yn_["AICP"]["Classifier"];
  for (YAML::const_iterator it = classificationNode.begin(); it != classificationNode.end(); ++it) {
    const std::string key = it->first.as<std::string>();

    if (key.compare("type") == 0) {
      classification_params.type = it->second.as<std::string>();
    }
  }

  if (classification_params.type.compare("SVM") == 0) {

    YAML::Node svmNode = classificationNode["SVM"];

    for(YAML::const_iterator it=svmNode.begin();it != svmNode.end();++it) {
      const std::string key = it->first.as<std::string>();

      if(key.compare("threshold") == 0) {
        classification_params.svm.threshold = it->second.as<double>();
      }
      else if(key.compare("trainingFile") == 0) {
        classification_params.svm.trainingFile = it->second.as<std::string>();
      }
      else if(key.compare("testingFile") == 0) {
          classification_params.svm.testingFile = it->second.as<std::string>();
      }
      else if(key.compare("saveFile") == 0) {
        classification_params.svm.saveFile = it->second.as<std::string>();
      }
      else if(key.compare("saveProbs") == 0) {
        classification_params.svm.saveProbs = it->second.as<std::string>();
      }
      else if(key.compare("modelLocation") == 0) {
        classification_params.svm.modelLocation = it->second.as<std::string>();
      }
    }
  }
  YAML::Node experimentsNode = yn_["AICP"]["Experiments"];
  for(YAML::const_iterator it=experimentsNode.begin();it != experimentsNode.end();++it) {

    const string key = it->first.as<string>();

    if(key.compare("type") == 0) {
      experiments_param = it->second.as<string>();
    }
  }

  cout << "============================" << endl
       << "Parsed YAML Config" << endl
       << "============================" << endl;

  cout << "[Main] Registration Type: "                 << registration_params.type                          << endl;
  cout << "[Main] Sensor Range: "                      << registration_params.sensorRange                   << endl;
  cout << "[Main] Sensor Angular View: "               << registration_params.sensorAngularView             << endl;
  cout << "[Main] Load Poses from: "                   << registration_params.loadPosesFrom                 << endl;
  cout << "[Main] Initial Transform: "                 << registration_params.initialTransform              << endl;
  cout << "[Main] Save Corrected Pose: "               << registration_params.saveCorrectedPose             << endl;
  cout << "[Main] Save Initialized Reading Cloud: "    << registration_params.saveInitializedReadingCloud   << endl;
  cout << "[Main] Save Registered Reading Cloud: "     << registration_params.saveRegisteredReadingCloud    << endl;
  cout << "[Main] Enable Lcm Visualization: "          << registration_params.enableLcmVisualization        << endl;

  if(registration_params.type.compare("Pointmatcher") == 0) {
    cout << "[Pointmatcher] Config File Name: "                << registration_params.pointmatcher.configFileName        << endl;
    cout << "[Pointmatcher] Print Registration Statistics: "   << registration_params.pointmatcher.printOutputStatistics << endl;
  }
  else if(registration_params.type.compare("GICP") == 0) {
  }

  cout << "[Main] Overlap Type: "                   << overlap_params.type                             << endl;

  if(overlap_params.type.compare("OctreeBased") == 0) {
    cout << "[OctreeBased] Octomap Resolution: "    << overlap_params.octree_based.octomapResolution   << endl;
  }

  cout << "[Main] Classification Type: "       << classification_params.type                    << endl;

  if(classification_params.type.compare("SVM") == 0) {
    cout << "[SVM] Acceptance Threshold: "    << classification_params.svm.threshold           << endl;
    cout << "[SVM] Training File: "           << classification_params.svm.trainingFile        << endl;
    cout << "[SVM] Testing File: "            << classification_params.svm.testingFile         << endl;
    cout << "[SVM] Saving Model To: "         << classification_params.svm.saveFile            << endl;
    cout << "[SVM] Saving Probs To: "         << classification_params.svm.saveProbs           << endl;
    cout << "[SVM] Loading Model From: "      << classification_params.svm.modelLocation       << endl;
  }

  cout << "[Main] Experiments Type: "               << experiments_param                               << endl;
  cout << "============================" << endl;



  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  App* app= new App(lcm, cl_cfg, ca_cfg,
                    registration_params, overlap_params,
                    classification_params, experiments_param);

  while(0 == lcm->handle());
}
