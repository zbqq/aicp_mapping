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
                         Eigen::Matrix4f &T)
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
    registr_->registerClouds(reference, reading, T);

    cout << "========================" << endl
         << "[AICP Core] Correction:" << endl
         << "========================" << endl
         << T << endl;

//    T =  T * initialT_;

//    cout << "============================" << endl
//         << "[AICP Core] Corrected Pose:" << endl
//         << "============================" << endl
//         << T << endl;
}

void App::operator()() {
    running_ = true;
    while (running_) {
        std::unique_lock<std::mutex> lock(worker_mutex_);
        // Wait for notification from planarLidarHandler
        worker_condition_.wait_for(lock, std::chrono::milliseconds(1000));

        // Copy current workload from cloud queue to work queue
        //    std::list<pcl::PointCloud<pcl::PointXYZ>::Ptr> work_queue;
        //    std::list<vector<LidarScan>> scans_queue;
        std::list<AlignedCloudPtr> work_queue;
        {
            std::unique_lock<std::mutex> lock(data_mutex_);
            while (!cloud_queue_.empty()) {
                work_queue.push_back(cloud_queue_.front());
                cloud_queue_.pop_front();
                //        scans_queue.push_back(scans_queue_.front());
                //        scans_queue_.pop_front();
            }
        }

        // Process workload
        for (auto cloud : work_queue) {
            // For storing current cloud
            //      SweepScan* current_sweep = new SweepScan();
            //      vector<LidarScan> first_sweep_scans_list = scans_queue.front();
            //      scans_queue.pop_front();

            // First point cloud (becomes first reference)
            if(aligned_clouds_graph_->isEmpty())
            {
                //        current_sweep->populateSweepScan(first_sweep_scans_list, *cloud, sweep_scans_list_->getNbClouds());
                //        sweep_scans_list_->initializeCollection(*current_sweep);
                // Initialize graph
                cloud->setReference();
                aligned_clouds_graph_->initialize(cloud);

                if (cl_cfg_.verbose)
                {
                    // Publish first reference cloud
                    reference_vis_ = aligned_clouds_graph_->getCurrentReference()->getCloud();
                    vis_->publishCloud(reference_vis_, 0, "First Reference");

                    // DEBUG: Save first reference cloud to file
                    stringstream first_ref;
                    first_ref << data_directory_path_.str();
                    first_ref << "/reference_";
                    first_ref << to_string(0);
                    first_ref << ".pcd";
                    pcd_writer_.write<pcl::PointXYZ> (first_ref.str (),
                                                      *(aligned_clouds_graph_->getLastCloud()->getCloud()),
                                                      false);
                }
            }
            else
            {
                TimingUtils::tic();

                if (cl_cfg_.verbose)
                {
                    // Publish original reading cloud
                    last_reading_vis_ = cloud->getCloud();
                    vis_->publishCloud(last_reading_vis_, 5000, "Original Reading");
                }

                /*===================================
                =           Set Input Clouds        =
                ===================================*/
                // Get current reference cloud
                pcl::PointCloud<pcl::PointXYZ>::Ptr reference = aligned_clouds_graph_->getCurrentReference()->getCloud();

                Eigen::Isometry3d ref_pose, read_pose;
                ref_pose = aligned_clouds_graph_->getCurrentReference()->getCorrectedPose();
                read_pose = cloud->getPriorPose();

                cout << "size before: " << cloud->getCloud()->size() << endl;
                cout << "prior pose before: " << cloud->getPriorPose().translation() << endl;
                cout << "corr pose before: " << cloud->getCorrectedPose().translation() << endl;
                cout << "correction before: " << cloud->getCorrection().translation() << endl;

                // Initialize clouds before sending to filters
                // (simulates correction integration only if "debug" mode)
                pcl::PointCloud<pcl::PointXYZ>::Ptr reading (new pcl::PointCloud<pcl::PointXYZ>);
                if (cl_cfg_.working_mode == "robot")
                    *reading = *(cloud->getCloud());
                else
                {
                    pcl::transformPointCloud (*(cloud->getCloud()), *reading, initialT_);
                    Eigen::Isometry3d initialT_iso = fromMatrix4fToIsometry3d(initialT_);
                    read_pose = initialT_iso * read_pose;
                    // update AlignedCloud with integrated prior pose
                    cloud->setPriorPose(read_pose);
                }

                if (cl_cfg_.verbose)
                {
                    // Publish initialized reading cloud
                    vis_->publishCloud(reading, 5010, "Initialized Reading");
                    // Publish current reference cloud
                    vis_->publishCloud(reference, 5020, "Current Reference");
                }


                cout << "size before 2: " << cloud->getCloud()->size() << endl;
                cout << "prior pose before 2: " << cloud->getPriorPose().translation() << endl;
                cout << "corr pose before 2: " << cloud->getCorrectedPose().translation() << endl;
                cout << "correction before 2: " << cloud->getCorrection().translation() << endl;

                /*===================================
                =        Filter Input Clouds        =
                ===================================*/

                pcl::PointCloud<pcl::PointXYZ> overlap_reference;
                pcl::PointCloud<pcl::PointXYZ> overlap_reading;
                if (cl_cfg_.failure_prediction_mode)
                {
                    // ------------------
                    // FOV-based Overlap
                    // ------------------
                    fov_overlap_ = overlapFilter(*reference, *reading,
                                                 ref_pose, read_pose,
                                                 reg_params_.sensorRange , reg_params_.sensorAngularView,
                                                 overlap_reference, overlap_reading);
                    cout << "====================================" << endl
                         << "[Main] FOV-based Overlap: " << fov_overlap_ << " %" << endl
                         << "====================================" << endl;
                }

                // ------------------------------------
                // Pre-filtering: 1) down-sampling
                //                2) planes extraction
                // ------------------------------------
                pcl::PointCloud<pcl::PointXYZ>::Ptr ref_prefiltered (new pcl::PointCloud<pcl::PointXYZ>);
                regionGrowingUniformPlaneSegmentationFilter(reference, ref_prefiltered);
                pcl::PointCloud<pcl::PointXYZ>::Ptr read_prefiltered (new pcl::PointCloud<pcl::PointXYZ>);
                regionGrowingUniformPlaneSegmentationFilter(reading, read_prefiltered);

                if (cl_cfg_.verbose)
                {
                    stringstream filtered_ref;
                    filtered_ref << data_directory_path_.str();
                    filtered_ref << "/reference_prefiltered.pcd";
                    pcd_writer_.write<pcl::PointXYZ> (filtered_ref.str (), *ref_prefiltered, false);
                    stringstream filtered_read;
                    filtered_read << data_directory_path_.str();
                    filtered_read << "/reading_prefiltered.pcd";
                    pcd_writer_.write<pcl::PointXYZ> (filtered_read.str (), *read_prefiltered, false);
                }

                // ---------------------
                // Octree-based Overlap
                // ---------------------
                ColorOcTree* ref_tree;
                ColorOcTree* read_tree = new ColorOcTree(overlap_params_.octree_based.octomapResolution);

                // 1) create octree from reference cloud (wrt robot's point of view)
                // 2) add the reading cloud and compute overlap
                ref_tree = overlapper_->computeOverlap(*ref_prefiltered, *read_prefiltered,
                                                       ref_pose, read_pose,
                                                       read_tree);
                octree_overlap_ = overlapper_->getOverlap();

                cout << "====================================" << endl
                     << "[Main] Octree-based Overlap: " << octree_overlap_ << " %" << endl
                     << "====================================" << endl;

                if (cl_cfg_.verbose)
                {
                    // TO DO: useOctomap is disabled/commented out in Director
                    // (director/src/python/director/startup.py) -> WHY?
                    // Publish reference octree
                    vis_->publishOctree(ref_tree, "OCTOMAP_REF");
                    // Publish reading octree
                    vis_->publishOctree(read_tree, "OCTOMAP");
                }

                if (cl_cfg_.failure_prediction_mode)
                {
                    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr matched_planes_reference (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
                    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr matched_planes_reading (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
                    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr eigenvectors (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
                    // -------------
                    // Alignability
                    // -------------
                    // Alignability computed on points belonging to the region of overlap (overlap_points_A, overlap_points_B)
                    alignability_ = alignabilityFilter(overlap_reference, overlap_reading,
                                                       ref_pose, read_pose,
                                                       matched_planes_reference, matched_planes_reading, eigenvectors);
                    cout << "====================================" << endl
                         << "[Main] Alignability: " << alignability_ << " % (degenerate if ~ 0)" << endl
                         << "====================================" << endl;

                    /*===================================
                    =           Classification          =
                    ===================================*/
                    // ---------------
                    // Alignment Risk
                    // ---------------
                    MatrixXd testing_data(1, 2);
                    testing_data << (float)octree_overlap_, (float)alignability_;

                    classifier_->test(testing_data, &risk_prediction_);
                    cout << "====================================" << endl
                         << "[Main] Alignment Risk: " << risk_prediction_ << " (0-1)" << endl
                         << "====================================" << endl;

                    if (cl_cfg_.verbose)
                    {
                        // Publish matched planes reference cloud
                        vis_->publishCloud(matched_planes_reference, 9, "Matches Reference");
                        // Publish matched planes reading cloud
                        vis_->publishCloud(matched_planes_reading, 11, "Matches Reading");
                        // Publish eigenvectors alignability
                        eigenvectors->points.resize(4);
                        vis_->publishCloud(eigenvectors, 13, "Eigenvectors Alignability");
                    }
                }

                /*===================================
                =          Register Clouds          =
                ===================================*/
                Eigen::Matrix4f correction = Eigen::Matrix4f::Identity(4,4);

                this->doRegistration(*ref_prefiltered, *read_prefiltered, correction);

                cout << "aligned_clouds_graph_->getNbClouds(): " << aligned_clouds_graph_->getNbClouds() << endl;

                int update_frequency = 50;
                if((!cl_cfg_.failure_prediction_mode &&
                    (aligned_clouds_graph_->getNbClouds() + 1) % update_frequency == 0))// ||
                   //(cl_cfg_.failure_prediction_mode && risk_prediction_(0,0) > class_params_.svm.threshold))
                {
                    cout << "BLA" << endl;
//                }
//                if(risk_prediction_(0,0) > class_params_.svm.threshold)
//                {
//                    cout << "====================================" << endl
//                         << "[Main] REFERENCE UPDATE" << endl
//                         << "====================================" << endl;
//                    // Reference Update Statistics
//                    updates_counter_ ++;

//                    int current_cloud_id = sweep_scans_list_->getCurrentCloud().getId();
//                    // Updating reference with current reading (non-aligned)
//                    if(sweep_scans_list_->getCloud(current_cloud_id).setReference())
//                    {
//                        sweep_scans_list_->getCloud(current_cloud_id).disableReference();
//                        cout << "SET REFERENCE aligned: " << current_cloud_id << endl;
//                    }
//                    else
//                    {
//                        current_sweep->populateSweepScan(first_sweep_scans_list, *read_ptr, sweep_scans_list_->getNbClouds(), -1, 1);
//                        sweep_scans_list_->addSweep(*current_sweep, initialT_);
//                        current_cloud_id = sweep_scans_list_->getCurrentCloud().getId();
//                        cout << "SET REFERENCE original: " << current_cloud_id << endl;
//                    }

//                    sweep_scans_list_->getCloud(current_cloud_id).setReference();
//                    sweep_scans_list_->updateReference(current_cloud_id);

//                    // To file
//                    stringstream ss_tmp3;
//                    ss_tmp3 << data_directory_path_.str();
//                    ss_tmp3 << "/ref_";
//                    ss_tmp3 << to_string(sweep_scans_list_->getCurrentReference().getId());
//                    ss_tmp3 << ".pcd";
//                    pcd_writer_.write<pcl::PointXYZ> (ss_tmp3.str (), *sweep_scans_list_->getCurrentReference().getCloud(), false);

//                    rejected_correction = TRUE;
                }
                else
                {
                    pcl::PointCloud<pcl::PointXYZ>::Ptr output (new pcl::PointCloud<pcl::PointXYZ>);
                    pcl::transformPointCloud (*reading, *output, correction);
                    Eigen::Isometry3d correction_iso = fromMatrix4fToIsometry3d(correction);
                    cloud->updateCloud(output, correction_iso, false, aligned_clouds_graph_->getCurrentReferenceId());

                    initialT_ = correction * initialT_;

                    Eigen::Isometry3d corr_pose = correction_iso * read_pose;
                    cout << "size after: " << cloud->getCloud()->size() << endl;
                    cout << "prior pose after: " << cloud->getPriorPose().translation() << endl;
                    cout << "corr pose after: " << cloud->getCorrectedPose().translation() << endl;
                    cout << "corr pose after 2: " << corr_pose.translation() << endl;
                    cout << "corr pose after 3: " << initialT_ << endl;
                    cout << "correction after: " << cloud->getCorrection().translation() << endl;
//                    current_sweep->populateSweepScan(first_sweep_scans_list, *output, sweep_scans_list_->getNbClouds(), sweep_scans_list_->getCurrentReference().getId(), enableRef);
//                    sweep_scans_list_->addSweep(*current_sweep, Ttot);

//                    current_correction_ = getTransfParamAsIsometry3d(Ttot);
//                    updated_correction_ = TRUE;

                    if (cl_cfg_.verbose)
                    {
                        // Publish aligned reading cloud
                        vis_->publishCloud(output, 5030, "Aligned Reading");
                        // Publish aligned reading cloud
                        last_reading_vis_ = cloud->getCloud();
                        vis_->publishCloud(last_reading_vis_, 5040, "Aligned Reading 2");
                    }
                }

                TimingUtils::toc();

//                // DEBUG
//                cout << "============================" << endl
//                     << "[Main] Statistics:" << endl
//                     << "============================" << endl;
//                cout << "REFERENCE: " << sweep_scans_list_->getCurrentReference().getId() << endl;
//                cout << "Cloud ID: " << sweep_scans_list_->getCurrentCloud().getId() << endl;
//                cout << "Number Clouds: " << sweep_scans_list_->getNbClouds() << endl;
//                cout << "Updates: " << updates_counter_ << endl;
            }

            //if (cl_cfg_.verbose)
            //  drawPointCloudCollections(lcm_, 5030, local_, *sweep_scans_list_->getCurrentCloud().getCloud(), 1, "Reading Aligned");

//            current_sweep->~SweepScan();

            cout << "--------------------------------------------------------------------------------------" << endl;
        }
    }
}
} // namespace aicp
