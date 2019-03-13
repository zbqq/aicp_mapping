#pragma once

#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <Eigen/Dense>

#include "aicp_registration/common.hpp"
#include "aicp_registration/pointmatcher_registration.hpp"
#include "aicp_registration/aligned_clouds_graph.hpp"
#include "aicp_registration/abstract_registrator.hpp"

#include "aicp_overlap/common.hpp"
#include "aicp_overlap/abstract_overlapper.hpp"

#include "aicp_classification/common.hpp"
#include "aicp_classification/abstract_classification.hpp"

#include "aicp_utils/visualizer.hpp"

struct CommandLineConfig
{
    string registration_config_file; //e.g. icp_autotuned.yaml
    string aicp_config_file;         //e.g. aicp_config.yaml
    string pose_body_channel;
    string output_channel;
    string working_mode;
    string fixed_frame;
    bool load_map_from_file;
    bool localize_against_prior_map;
    bool localize_against_built_map;
    string map_from_file_path;
    float crop_map_around_base;
    bool merge_aligned_clouds_to_map;
    bool failure_prediction_mode;
    int reference_update_frequency;
    float max_correction_magnitude;
    bool verbose;
};

namespace aicp {

class App{
public:
    App(const CommandLineConfig& cl_cfg,
        RegistrationParams reg_params,
        OverlapParams overlap_params,
        ClassificationParams class_params);

    inline virtual ~App(){
        delete aligned_clouds_graph_;
        delete prior_map_;
        delete vis_;
    }

    RegistrationParams reg_params_;
    OverlapParams overlap_params_;
    ClassificationParams class_params_;
    int online_results_line_;

    // thread function doing actual work
    void operator()();

    // AICP core pipeline
    void filterCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
                     pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out);
    void runAicpPipeline(pcl::PointCloud<pcl::PointXYZ>::Ptr& reference_prefiltered,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr& reading_prefiltered,
                         Eigen::Isometry3d& reference_pose,
                         Eigen::Isometry3d& reading_pose,
                         Eigen::Matrix4f &T);

    // Gets
    float getFOVOverlap() {
        return fov_overlap_;
    }

    float getOctreeOverlap() {
        return octree_overlap_;
    }

    float getAlignability() {
        return alignability_;
    }

    Eigen::MatrixXd& getAlignmentRisk() {
        return risk_prediction_;
    }

    const string getDataDirectoryPath(){
        return data_directory_path_.str();
    }

private:
    // App specific
    void setReference(AlignedCloudPtr& reading_cloud,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr& reference_cloud,
                      Eigen::Isometry3d& reference_pose);
    // App specific
    void setAndFilterReading(AlignedCloudPtr& reading_cloud_in,
                             pcl::PointCloud<pcl::PointXYZ>::Ptr& reading_cloud_out,
                             Eigen::Isometry3d& reading_pose);

    void computeOverlap(pcl::PointCloud<pcl::PointXYZ>::Ptr& reference_cloud,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr& reading_cloud,
                        Eigen::Isometry3d& reference_pose,
                        Eigen::Isometry3d& reading_pose);
    void computeAlignmentRisk(pcl::PointCloud<pcl::PointXYZ>::Ptr& reference_cloud,
                              pcl::PointCloud<pcl::PointXYZ>::Ptr& reading_cloud,
                              Eigen::Isometry3d& reference_pose,
                              Eigen::Isometry3d& reading_pose);
    void computeRegistration(pcl::PointCloud<pcl::PointXYZ>& reference_cloud,
                             pcl::PointCloud<pcl::PointXYZ>& reading_cloud,
                             Eigen::Matrix4f &T);

protected:
    void paramInit(){
        pose_initialized_ = false;
        updated_correction_ = true;

        map_initialized_ = false;
        pose_marker_initialized_ = false;

        first_cloud_initialized_ = false;

        valid_correction_ = false;
        force_reference_update_ = false;

        // Reference cloud update counters
        updates_counter_ = 0;

        // Count lines output file
        online_results_line_ = 0;

        clear_clouds_buffer_ = false;

        fov_overlap_ = -1.0;
        octree_overlap_ = -1.0;
        alignability_ = -1.0;

        // Initialize reading with previous correction when "debug" mode
        initialT_ = Eigen::Matrix4f::Identity(4,4);

        local_ = Eigen::Isometry3d::Identity();
        world_to_body_msg_ = Eigen::Isometry3d::Identity();
        world_to_body_marker_msg_ = Eigen::Isometry3d::Identity();
        corrected_pose_ = Eigen::Isometry3d::Identity(); // world_to_body corrected (world -> body)
        total_correction_ = Eigen::Isometry3d::Identity();
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
    std::list<AlignedCloudPtr> cloud_queue_;
    std::mutex data_mutex_;
    std::mutex robot_state_mutex_;
    std::mutex robot_behavior_mutex_;
    std::mutex cloud_accumulate_mutex_;

    // Data structure
    AlignedCloudsGraph* aligned_clouds_graph_;
    // Map
    AlignedCloud* prior_map_;
    pcl::PointCloud<pcl::PointXYZ> aligned_map_;
    // Visualizer
    Visualizer* vis_;

    Eigen::Matrix4f initialT_;

    // Transformation matrices
    Eigen::Isometry3d local_;
    Eigen::Isometry3d world_to_body_msg_; // Captures the position of the body frame in world from launch
                                          // without drift correction

    bool pose_initialized_;
    bool updated_correction_;
    // Map service
    bool map_initialized_;
    bool pose_marker_initialized_;
    Eigen::Isometry3d world_to_body_marker_msg_; // Initializes body pose in map from user interaction marker
    bool first_cloud_initialized_;

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
    Eigen::MatrixXd risk_prediction_;

    // Correction variables
    bool valid_correction_;
    bool force_reference_update_;
    Eigen::Isometry3d corrected_pose_;
    Eigen::Isometry3d total_correction_;
    std::vector<Eigen::Isometry3d> poseNodes_;
    // Reference cloud update counters
    int updates_counter_;
    // Current reference pre-filtered
    pcl::PointCloud<pcl::PointXYZ>::Ptr ref_prefiltered;

    // DEBUG: Write to file
    pcl::PCDWriter pcd_writer_;
    // Visualization
    pcl::PointCloud<pcl::PointXYZ>::Ptr reference_vis_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr last_reading_vis_;
    //Eigen::Isometry3d ref_pose_vis_;
};
} // namespace aicp
