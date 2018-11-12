#pragma once
#include <memory>
#include <lcm/lcm-cpp.hpp>
#include <bot_param/param_client.h>
#include <cloud_accumulate/cloud_accumulate.hpp>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <lcmtypes/bot_core/pose_t.hpp>
#include <lcmtypes/bot_core/rigid_transform_t.hpp>

#include "aicp_registration/common.hpp"
#include "aicp_classification/common.hpp"
#include "aicp_overlap/common.hpp"
#include "aicp_registration/aligned_sweeps_collection.hpp"
#include "aicp_registration/pointmatcher_registration.hpp"

#include "aicp_registration/abstract_registrator.hpp"
#include "aicp_overlap/abstract_overlapper.hpp"
#include "aicp_classification/abstract_classification.hpp"

#include <pointmatcher/PointMatcher.h>

struct CommandLineConfig
{
  string configFile;
  string pose_body_channel;
  string output_channel;
  string working_mode;
  int failure_prediction_mode;
  bool verbose;
  bool apply_correction;
};
namespace aicp {
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
    int online_results_line_;

    // thread function for doing actual work
    void operator()();
  protected:
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
    std::vector<LidarScan> lidar_scans_list_;
    AlignedSweepsCollection* sweep_scans_list_;

    Eigen::Matrix4f initialT_;

    // Transformation matrices
    Eigen::Isometry3d local_;
    Eigen::Isometry3d body_to_lidar_; // Variable tf from the lidar to the robot's base link
    Eigen::Isometry3d head_to_lidar_; // Fixed tf from the lidar to the robot's head frame
    Eigen::Isometry3d world_to_body_msg_; // Captures the position of the body frame in world from launch
                                          // without drift correction
    long long int world_to_body_msg_utime_;
    long long int world_to_body_msg_utime_first_;
    Eigen::Isometry3d world_to_head_now_; // running head position estimate
    Eigen::Isometry3d world_to_body_corr_first_;

    bool pose_initialized_;
    bool updated_correction_;
    bool rejected_correction;

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
    std::vector<float> other_predictions_; // state of the art

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
} // namespace aicp
