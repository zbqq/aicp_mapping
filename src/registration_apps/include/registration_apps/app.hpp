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

#include "aicp_registration/pointmatcher_registration.hpp"
#include "aicp_registration/common.hpp"
#include "aicp_classification/common.hpp"
#include "aicp_overlap/common.hpp"

#include "aicp_registration/aligned_sweeps_collection.hpp"
#include "aicp_registration/aligned_clouds_graph.hpp"

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
    App(const CommandLineConfig& cl_cfg,
        RegistrationParams reg_params,
        OverlapParams overlap_params,
        ClassificationParams class_params,
        string exp_params);


    ~App(){
    }
    RegistrationParams reg_params_;
    OverlapParams overlap_params_;
    ClassificationParams class_params_;
    string exp_params_;
    int online_results_line_;

    // thread function doing actual work
    void operator()();

protected:
    void paramInit(){
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
//        world_to_body_corr_first_ = Eigen::Isometry3d::Identity();
        corrected_pose_ = Eigen::Isometry3d::Identity(); // world_to_body corrected (world -> body)
        current_correction_ = Eigen::Isometry3d::Identity();
    }

    const CommandLineConfig cl_cfg_;

    std::unique_ptr<AbstractRegistrator> registr_;
    std::unique_ptr<AbstractOverlapper> overlapper_;
    std::unique_ptr<AbstractClassification> classifier_;

    // Thread variables
    bool running_;
    std::thread worker_thread_;
    std::condition_variable worker_condition_;
    std::mutex worker_mutex_;
//    std::list<pcl::PointCloud<pcl::PointXYZ>::Ptr> data_queue_;
    std::list<AlignedCloudPtr> cloud_queue_;
    std::mutex data_mutex_;
    std::mutex robot_state_mutex_;
    std::mutex robot_behavior_mutex_;
    std::mutex cloud_accumulate_mutex_;

    // Data structure
    AlignedCloudsGraph* aligned_clouds_graph_;

    Eigen::Matrix4f initialT_;

    // Transformation matrices
    Eigen::Isometry3d local_;
//    Eigen::Isometry3d body_to_lidar_; // Variable tf from the lidar to the robot's base link
//    Eigen::Isometry3d head_to_lidar_; // Fixed tf from the lidar to the robot's head frame
    Eigen::Isometry3d world_to_body_msg_; // Captures the position of the body frame in world from launch
                                          // without drift correction
//    long long int world_to_body_msg_utime_;
//    long long int world_to_body_msg_utime_first_;
//    Eigen::Isometry3d world_to_head_; // running head position estimate
//    Eigen::Isometry3d world_to_body_corr_first_;

    bool pose_initialized_;
    bool updated_correction_;
    bool rejected_correction;

    // Clear buffer after pose update (to avoid distortion)
    bool clear_clouds_buffer_;

    // Create data folder for debug
    std::stringstream data_directory_path_;

    // AICP Parameters
    // Overlap
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

    // DEBUG: Write to file
    pcl::PCDWriter pcd_writer_;
    // Visualization
    pcl::PointCloud<pcl::PointXYZ>::Ptr reference_vis_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr last_reading_vis_;

    // Init handlers
    void doRegistration(pcl::PointCloud<pcl::PointXYZ>& reference, pcl::PointCloud<pcl::PointXYZ>& reading,
                        Eigen::Matrix4f &T, vector<float>& failure_prediction_factors);

    // Getters

};
} // namespace aicp
