#include "registration_apps/app.hpp"
#include "aicp_common_utils/timing.hpp"
#include "aicp_registration/registration.hpp"
#include "aicp_overlap/overlap.hpp"
#include "aicp_classification/classification.hpp"
#include "aicp_common_utils/common.hpp"
#include "aicp_drawing_utils/drawingUtils.hpp"

#include <lcmtypes/bot_core/double_array_t.hpp>


namespace aicp {

App::App(const CommandLineConfig& cl_cfg,
         RegistrationParams reg_params,
         OverlapParams overlap_params,
         ClassificationParams class_params,
         string exp_params) :
    cl_cfg_(cl_cfg),
    reg_params_(reg_params), overlap_params_(overlap_params),
    class_params_(class_params), exp_params_(exp_params)
{

}

void App::doRegistration(pcl::PointCloud<pcl::PointXYZ>& reference,
                         pcl::PointCloud<pcl::PointXYZ>& reading,
                         Eigen::Matrix4f &T,
                         vector<float>& failure_prediction_factors)
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
//    std::list<pcl::PointCloud<pcl::PointXYZ>::Ptr> work_queue;
//    std::list<vector<LidarScan>> scans_queue;
    std::list<AlignedCloud> work_queue;
    {
      std::unique_lock<std::mutex> lock(data_mutex_);
      while (!cloud_queue_.empty()) {
        work_queue.push_back(cloud_queue_.front());
        cloud_queue_.pop_front();
//        scans_queue.push_back(scans_queue_.front());
//        scans_queue_.pop_front();
      }
    }

    // process workload
    for (auto cloud : work_queue) {
      // For storing current cloud
//      SweepScan* current_sweep = new SweepScan();
//      vector<LidarScan> first_sweep_scans_list = scans_queue.front();
//      scans_queue.pop_front();

      // first point cloud
      if(aligned_clouds_graph_->isEmpty())
      {
//        current_sweep->populateSweepScan(first_sweep_scans_list, *cloud, sweep_scans_list_->getNbClouds());
//        sweep_scans_list_->initializeCollection(*current_sweep);
          aligned_clouds_graph_->initialize(cloud);

        //if (cl_cfg_.verbose)
        //{
        //  // To director first reference cloud (Point Cloud 0, Frame 0)
        //  drawPointCloudCollections(lcm_, 0, local_, *cloud, 1);
        //}

        // To file
        stringstream ss_ref;
        ss_ref << data_directory_path_.str();
        ss_ref << "/ref_";
        ss_ref << to_string(0);
        ss_ref << ".pcd";
        pcd_writer_.write<pcl::PointXYZ> (ss_ref.str (), *(cloud.getCloud()), false);
      }
      else
      {
//        TimingUtils::tic();

//        // TODO move to LCM implementation
//        //if (cl_cfg_.verbose)
//        //{
//        //  // Publish clouds in Director
//        //  drawPointCloudCollections(lcm_, 5000, local_, *cloud, 1, "Reading Original");
//        //}

//        /*===================================
//        =           Set Input Clouds        =
//        ===================================*/
//        // Get current reference cloud
//        pcl::PointCloud<pcl::PointXYZ>::Ptr ref = sweep_scans_list_->getCurrentReference().getCloud();

//        // To file
//        stringstream ss_ref;
//        ss_ref << data_directory_path_.str();
//        ss_ref << "/from_get_";
//        ss_ref << to_string(0);
//        ss_ref << ".pcd";
//        pcd_writer_.write<pcl::PointXYZ> (ss_ref.str (), *ref, false);

//        Eigen::Isometry3d ref_pose, read_pose;
//        ref_pose = sweep_scans_list_->getCurrentReference().getBodyPose();
//        read_pose = first_sweep_scans_list.back().getBodyPose();

//        // Initialize clouds before sending to filters
//        // (simulates correction integration)
//        pcl::PointCloud<pcl::PointXYZ>::Ptr ref_ptr (new pcl::PointCloud<pcl::PointXYZ>);
//        pcl::PointCloud<pcl::PointXYZ>::Ptr read_ptr (new pcl::PointCloud<pcl::PointXYZ>);
//        if (cl_cfg_.apply_correction && cl_cfg_.working_mode != "robot")
//        {
//          pcl::transformPointCloud (*cloud, *read_ptr, initialT_);
//          Eigen::Isometry3d initialT_iso = fromMatrix4fToIsometry3d(initialT_);
//          read_pose = initialT_iso * read_pose;
//        }
//        else
//          *read_ptr = *cloud;

//        ref_ptr = ref;
//        // TODO move to LCM implementation
//        // if (cl_cfg_.verbose)
//        // {
//        //   // Publish clouds in Director
//        //   drawPointCloudCollections(lcm_, 5010, local_, *read_ptr, 1, "Reading Initialized");
//        //   drawPointCloudCollections(lcm_, 5020, local_, *ref_ptr, 1, "Reference");
//        // }

//        /*===================================
//        =        Filter Input Clouds        =
//        ===================================*/

//        pcl::PointCloud<pcl::PointXYZ>::Ptr ref_xyz_ptr (new pcl::PointCloud<pcl::PointXYZ>);
//        pcl::PointCloud<pcl::PointXYZ>::Ptr read_xyz_ptr (new pcl::PointCloud<pcl::PointXYZ>);
//        copyPointCloud(*ref_ptr, *ref_xyz_ptr);
//        copyPointCloud(*read_ptr, *read_xyz_ptr);

//        pcl::PointCloud<pcl::PointXYZ> overlap_points_A;
//        pcl::PointCloud<pcl::PointXYZ> overlap_points_B;
//        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudA_matched_planes (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
//        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudB_matched_planes (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
//        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr eigenvectors (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

//        // ------------------
//        // FOV-based Overlap
//        // ------------------
//        fov_overlap_ = overlapFilter(*ref_xyz_ptr, *read_xyz_ptr,
//                                     ref_pose, read_pose,
//                                     reg_params_.sensorRange , reg_params_.sensorAngularView,
//                                     overlap_points_A, overlap_points_B);
//        cout << "====================================" << endl
//             << "[Main] FOV-based Overlap: " << fov_overlap_ << " %" << endl
//             << "====================================" << endl;

//        // ------------------------------------
//        // Pre-filtering: 1) down-sampling
//        //                2) planes extraction
//        // ------------------------------------
//        pcl::PointCloud<pcl::PointXYZ>::Ptr ref_xyz_prefiltered (new pcl::PointCloud<pcl::PointXYZ>);
//        regionGrowingUniformPlaneSegmentationFilter(ref_xyz_ptr, ref_xyz_prefiltered);
//        pcl::PointCloud<pcl::PointXYZ>::Ptr read_xyz_prefiltered (new pcl::PointCloud<pcl::PointXYZ>);
//        regionGrowingUniformPlaneSegmentationFilter(read_xyz_ptr, read_xyz_prefiltered);

//        stringstream ss_tmp1;
//        ss_tmp1 << data_directory_path_.str();
//        ss_tmp1 << "/point_cloud_A_prefiltered.pcd";
//        pcd_writer_.write<pcl::PointXYZ> (ss_tmp1.str (), *ref_xyz_prefiltered, false);
//        stringstream ss_tmp2;
//        ss_tmp2 << data_directory_path_.str();
//        ss_tmp2 << "/point_cloud_B_prefiltered.pcd";
//        pcd_writer_.write<pcl::PointXYZ> (ss_tmp2.str (), *read_xyz_prefiltered, false);

//        // ---------------------
//        // Octree-based Overlap
//        // ---------------------
//        ColorOcTree* ref_tree;
//        ColorOcTree* read_tree = new ColorOcTree(overlap_params_.octree_based.octomapResolution);

//        // Create octree from reference cloud (wrt robot point of view),
//        // add the reading cloud and compute overlap
//        ref_tree = overlapper_->computeOverlap(*ref_xyz_prefiltered, *read_xyz_prefiltered,
//                                               ref_pose, read_pose,
//                                               read_tree);
//        octree_overlap_ = overlapper_->getOverlap();

//        cout << "====================================" << endl
//             << "[Main] Octree-based Overlap: " << octree_overlap_ << " %" << endl
//             << "====================================" << endl;

//        // -------------
//        // Alignability
//        // -------------
//        // Alignability computed on points belonging to the region of overlap (overlap_points_A, overlap_points_B)
//        alignability_ = alignabilityFilter(overlap_points_A, overlap_points_B,
//                                           ref_pose, read_pose,
//                                           cloudA_matched_planes, cloudB_matched_planes, eigenvectors);
//        cout << "[Main] Alignability (degenerate if ~ 0): " << alignability_ << " %" << endl;

//        /*===================================
//        =           Classification          =
//        ===================================*/
//        // ---------------
//        // Alignment Risk
//        // ---------------
//        MatrixXd testing_data(1, 2);
//        testing_data << (float)octree_overlap_, (float)alignability_;

//        classifier_->test(testing_data, &risk_prediction_);
//        std::cout << "[Main] Alignment Risk Prediction (0-1): " << risk_prediction_ << std::endl;

//        if (cl_cfg_.verbose)
//        {
//          /*===================================
//          =     Visualize Octree Overlap      =
//          ===================================*/
////          publishOctreeToLCM(lcm_, ref_tree, "OCTOMAP_REF");
////          publishOctreeToLCM(lcm_, read_tree, "OCTOMAP");
//          /*===================================
//          =      Visualize Alignability       =
//          ===================================*/
////          drawPointCloudNormalsCollections(lcm_, 9, local_, *cloudA_matched_planes, 0, "Matches A");
////          drawPointCloudNormalsCollections(lcm_, 11, local_, *cloudB_matched_planes, 0, "Matches B");
////          eigenvectors->points.resize(4);
////          drawPointCloudNormalsCollections(lcm_, 13, local_, *eigenvectors, 0, "Alignability Eigenvectors");
//        }

//        /*===================================
//        =          Register Clouds          =
//        ===================================*/
//        Eigen::Matrix4f Ttot = Eigen::Matrix4f::Identity(4,4);

//        this->doRegistration(*ref_xyz_prefiltered, *read_xyz_prefiltered,
//                             Ttot, other_predictions_);

//        if((cl_cfg_.failure_prediction_mode == 0 && risk_prediction_(0,0) > class_params_.svm.threshold) ||
//           (cl_cfg_.failure_prediction_mode == 1 && other_predictions_.at(0) < 0.05))
//        {
//            cout << "====================================" << endl
//                 << "[Main] REFERENCE UPDATE" << endl
//                 << "====================================" << endl;
//          // Reference Update Statistics
//          updates_counter_ ++;

//          int current_cloud_id = sweep_scans_list_->getCurrentCloud().getId();
//          // Updating reference with current reading (non-aligned)
//          if(sweep_scans_list_->getCloud(current_cloud_id).setReference())
//          {
//            sweep_scans_list_->getCloud(current_cloud_id).disableReference();
//            cout << "SET REFERENCE aligned: " << current_cloud_id << endl;
//          }
//          else
//          {
//            current_sweep->populateSweepScan(first_sweep_scans_list, *read_ptr, sweep_scans_list_->getNbClouds(), -1, 1);
//            sweep_scans_list_->addSweep(*current_sweep, initialT_);
//            current_cloud_id = sweep_scans_list_->getCurrentCloud().getId();
//            cout << "SET REFERENCE original: " << current_cloud_id << endl;
//          }

//          sweep_scans_list_->getCloud(current_cloud_id).setReference();
//          sweep_scans_list_->updateReference(current_cloud_id);

//          // To file
//          stringstream ss_tmp3;
//          ss_tmp3 << data_directory_path_.str();
//          ss_tmp3 << "/ref_";
//          ss_tmp3 << to_string(sweep_scans_list_->getCurrentReference().getId());
//          ss_tmp3 << ".pcd";
//          pcd_writer_.write<pcl::PointXYZ> (ss_tmp3.str (), *sweep_scans_list_->getCurrentReference().getCloud(), false);

//          rejected_correction = TRUE;
//        }
//        else
//        {
//          bool enableRef = TRUE;
//          pcl::PointCloud<pcl::PointXYZ>::Ptr output (new pcl::PointCloud<pcl::PointXYZ>);
//          pcl::transformPointCloud (*cloud, *output, Ttot);
//          current_sweep->populateSweepScan(first_sweep_scans_list, *output, sweep_scans_list_->getNbClouds(), sweep_scans_list_->getCurrentReference().getId(), enableRef);
//          sweep_scans_list_->addSweep(*current_sweep, Ttot);

//          current_correction_ = getTransfParamAsIsometry3d(Ttot);
//          updated_correction_ = TRUE;

//          initialT_ = Ttot;
//        }

//        TimingUtils::toc();

//        // DEBUG
//        cout << "============================" << endl
//             << "[Main] Statistics:" << endl
//             << "============================" << endl;
//        cout << "REFERENCE: " << sweep_scans_list_->getCurrentReference().getId() << endl;
//        cout << "Cloud ID: " << sweep_scans_list_->getCurrentCloud().getId() << endl;
//        cout << "Number Clouds: " << sweep_scans_list_->getNbClouds() << endl;
//        cout << "Updates: " << updates_counter_ << endl;
      }

//      //if (cl_cfg_.verbose)
//      //  drawPointCloudCollections(lcm_, 5030, local_, *sweep_scans_list_->getCurrentCloud().getCloud(), 1, "Reading Aligned");

//      current_sweep->~SweepScan();

//      cout << "--------------------------------------------------------------------------------------" << endl;
    }
  }
}


} // namespace aicp
