#include "registration_apps/app.hpp"
#include "aicp_registration/registration.hpp"
#include "aicp_overlap/overlap.hpp"
#include "aicp_classification/classification.hpp"

#include "aicp_common_utils/timing.hpp"
#include "aicp_common_utils/common.hpp"
//#include "aicp_drawing_utils/drawingUtils.hpp"

namespace aicp {

App::App(const CommandLineConfig& cl_cfg,
         RegistrationParams reg_params,
         OverlapParams overlap_params,
         ClassificationParams class_params) :
    cl_cfg_(cl_cfg), reg_params_(reg_params),
    overlap_params_(overlap_params), class_params_(class_params)
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
}

void App::operator()() {
    running_ = true;
    while (running_) {
        std::unique_lock<std::mutex> lock(worker_mutex_);
        // Wait for notification from planarLidarHandler
        worker_condition_.wait_for(lock, std::chrono::milliseconds(1000));

        // Copy current workload from cloud queue to work queue
        std::list<AlignedCloudPtr> work_queue;
        {
            std::unique_lock<std::mutex> lock(data_mutex_);
            while (!cloud_queue_.empty()) {
                work_queue.push_back(cloud_queue_.front());
                cloud_queue_.pop_front();
            }
        }

        // Process workload
        for (auto cloud : work_queue) {

            // First point cloud (becomes first reference)
            if(!cl_cfg_.localize_against_prior_map &&
               !cl_cfg_.load_map_from_file &&
               aligned_clouds_graph_->isEmpty())
            {
                /*===================================
                =            First Cloud            =
                ===================================*/
                // Pre-filter first cloud
                pcl::PointCloud<pcl::PointXYZ>::Ptr ref_prefiltered (new pcl::PointCloud<pcl::PointXYZ>);
                regionGrowingUniformPlaneSegmentationFilter(cloud->getCloud(), ref_prefiltered);
                // update AlignedCloud
                cloud->updateCloud(ref_prefiltered, true);
                // Initialize graph
                aligned_clouds_graph_->initialize(cloud);

                // Publish first reference cloud
                reference_vis_ = aligned_clouds_graph_->getCurrentReference()->getCloud();
//                    vis_->publishCloud(reference_vis_, 0, "First Reference"); // TODO: update to unique template LCM - ROS
                vis_->publishCloud(reference_vis_, 0, "", cloud->getUtime());
                vis_->publishPose(aligned_clouds_graph_->getCurrentReference()->getCorrectedPose(), 0, "",
                                  cloud->getUtime());
                // Output map
                aligned_map_ = aligned_map_ + *reference_vis_;
                pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_map_ptr = aligned_map_.makeShared();
                vis_->publishMap(aligned_map_ptr, cloud->getUtime(), 1);

                first_cloud_initialized_ = true;
            }
            else {
                TimingUtils::tic();

                if (cl_cfg_.verbose)
                {
                    // Publish original reading cloud
                    last_reading_vis_ = cloud->getCloud();
//                    vis_->publishCloud(last_reading_vis_, 5000, "Original Reading"); // TODO: update to unique template LCM - ROS
//                    vis_->publishCloud(last_reading_vis_, 10, "/aicp/original_reading", cloud->getUtime());

                }

                /*===================================
                =          Set Input Clouds         =
                ===================================*/
                // Set reading cloud
                Eigen::Isometry3d read_pose;
                read_pose = cloud->getPriorPose();

                // Initialize cloud before sending to filters
                // (simulates correction integration only if "debug" mode
                // and integrates interactive marker pose)
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

                // Crop map
                pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_map (new pcl::PointCloud<pcl::PointXYZ>);
                if (cl_cfg_.load_map_from_file || cl_cfg_.localize_against_prior_map)
                {
                    // crop map around cloud->getPriorPose();
                    *cropped_map = *(prior_map_->getCloud());
                    Eigen::Matrix4f tmp = (cloud->getPriorPose()).matrix().cast<float>();
                    getPointsInOrientedBox(cropped_map,
                                           -cl_cfg_.crop_map_around_base,
                                           cl_cfg_.crop_map_around_base, tmp);
                }

                // Set reference cloud
                Eigen::Isometry3d ref_pose;
                pcl::PointCloud<pcl::PointXYZ>::Ptr ref_prefiltered;
                if (!first_cloud_initialized_ || cl_cfg_.localize_against_prior_map)
                {
                    ref_prefiltered = cropped_map;
                    ref_pose = cloud->getPriorPose();
                    first_cloud_initialized_ = true;
                }
                else if (cl_cfg_.localize_against_built_map)
                {
                    copyPointCloud(aligned_map_, *cropped_map);
                    Eigen::Matrix4f tmp = (cloud->getPriorPose()).matrix().cast<float>();
                    getPointsInOrientedBox(cropped_map,
                                           -cl_cfg_.crop_map_around_base,
                                           cl_cfg_.crop_map_around_base, tmp);
                    ref_prefiltered = cropped_map;
                    ref_pose = cloud->getPriorPose();
                }
                else
                {
                    ref_prefiltered = aligned_clouds_graph_->getCurrentReference()->getCloud();
                    ref_pose = aligned_clouds_graph_->getCurrentReference()->getCorrectedPose();
                }

                // Publish initialized reading cloud
//                    vis_->publishCloud(reading, 5010, "Initialized Reading"); // TODO: update to unique template LCM - ROS
                // Publish current reference cloud
//                    vis_->publishCloud(reference, 5020, "Current Reference");

                /*===================================
                =        Filter Input Clouds        =
                ===================================*/

                // ------------------------------------
                // Pre-filtering: 1) down-sampling
                //                2) planes extraction
                // ------------------------------------
                pcl::PointCloud<pcl::PointXYZ>::Ptr read_prefiltered (new pcl::PointCloud<pcl::PointXYZ>);
                regionGrowingUniformPlaneSegmentationFilter(reading, read_prefiltered);

                if (cl_cfg_.verbose)
                {
                    // Save filtered clouds to file
                    stringstream filtered_ref;
                    filtered_ref << data_directory_path_.str();
                    filtered_ref << "/reference_prefiltered.pcd";
                    pcd_writer_.write<pcl::PointXYZ> (filtered_ref.str (), *ref_prefiltered, false);
                    stringstream filtered_read;
                    filtered_read << data_directory_path_.str();
                    filtered_read << "/reading_prefiltered.pcd";
                    pcd_writer_.write<pcl::PointXYZ> (filtered_read.str (), *read_prefiltered, false);
                }

                pcl::PointCloud<pcl::PointXYZ> overlap_reference;
                pcl::PointCloud<pcl::PointXYZ> overlap_reading;
                if (cl_cfg_.failure_prediction_mode)
                {
                    // ------------------
                    // FOV-based Overlap
                    // ------------------
                    fov_overlap_ = overlapFilter(*ref_prefiltered, *read_prefiltered,
                                                 ref_pose, read_pose,
                                                 reg_params_.sensorRange , reg_params_.sensorAngularView,
                                                 overlap_reference, overlap_reading);
                    cout << "====================================" << endl
                         << "[Main] FOV-based Overlap: " << fov_overlap_ << " %" << endl
                         << "====================================" << endl;
                }


                // ---------------------
                // Octree-based Overlap
                // ---------------------
                ColorOcTree* ref_tree;
                ColorOcTree* read_tree = new ColorOcTree(overlap_params_.octree_based.octomapResolution);

//                if(//(cl_cfg_.load_map_from_file && aligned_clouds_graph_->getNbClouds() == 0) ||
//                    cl_cfg_.localize_against_prior_map)
//                    octree_overlap_ = 100.0;
//                else
//                {
                    // 1) create octree from reference cloud (wrt robot's point of view)
                    // 2) add the reading cloud and compute overlap
                    ref_tree = overlapper_->computeOverlap(*ref_prefiltered, *read_prefiltered,
                                                           ref_pose, read_pose,
                                                           read_tree);
                    octree_overlap_ = overlapper_->getOverlap();
//                }

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
                    Eigen::MatrixXd testing_data(1, 2);
                    testing_data << (float)octree_overlap_, (float)alignability_;

                    classifier_->test(testing_data, &risk_prediction_);
                    cout << "====================================" << endl
                         << "[Main] Alignment Risk: " << risk_prediction_ << " (0-1)" << endl
                         << "====================================" << endl;

                    if (cl_cfg_.verbose)
                    {
                        // Publish matched planes reference cloud
//                        vis_->publishCloud(matched_planes_reference, 9, "Matches Reference");
                        // Publish matched planes reading cloud
//                        vis_->publishCloud(matched_planes_reading, 11, "Matches Reading");
                        // Publish eigenvectors alignability
                        eigenvectors->points.resize(4);
//                        vis_->publishCloud(eigenvectors, 13, "Eigenvectors Alignability");
                    }
                }

                /*===================================
                =          Register Clouds          =
                ===================================*/
                Eigen::Matrix4f correction = Eigen::Matrix4f::Identity(4,4);
                pcl::PointCloud<pcl::PointXYZ>::Ptr output (new pcl::PointCloud<pcl::PointXYZ>);

                if(!cl_cfg_.failure_prediction_mode ||                      // if alignment risk disabled
                   risk_prediction_(0,0) <= class_params_.svm.threshold)    // or below threshold
                {
                    this->doRegistration(*ref_prefiltered, *read_prefiltered, correction);
                    // Probably failed alignment
                    if ((abs(correction(0,3)) > cl_cfg_.max_correction_magnitude ||
                         abs(correction(1,3)) > cl_cfg_.max_correction_magnitude ||
                         abs(correction(2,3)) > cl_cfg_.max_correction_magnitude) &&
                         aligned_clouds_graph_->getNbClouds() != 0)
                    {
                        cout << "[Main] -----> WRONG ALIGNMENT: DROPPED POINT CLOUD" << endl;
                        break;
                    }

                    pcl::transformPointCloud (*read_prefiltered, *output, correction);
                    Eigen::Isometry3d correction_iso = fromMatrix4fToIsometry3d(correction);
                    // update AlignedCloud with corrected pose and (prefiltered) cloud after alignment
                    cloud->updateCloud(output, correction_iso, false, aligned_clouds_graph_->getCurrentReferenceId());
                    // add AlignedCloud to graph
                    aligned_clouds_graph_->addCloud(cloud);

                    // windowed reference update policy (count number of clouds after last reference)
                    if(((aligned_clouds_graph_->getNbClouds() - (aligned_clouds_graph_->getCurrentReferenceId()+1))
                        % cl_cfg_.reference_update_frequency == 0) &&
                        !cl_cfg_.localize_against_prior_map)
                    {
                        // set AlignedCloud to be next reference
                        aligned_clouds_graph_->updateReference(aligned_clouds_graph_->getNbClouds()-1);
                        updates_counter_ ++;
                        cout << "[Main] -----> FREQUENCY REFERENCE UPDATE" << endl;
                    }
                    else if(cl_cfg_.load_map_from_file &&
                            !cl_cfg_.localize_against_prior_map &&
                            aligned_clouds_graph_->getNbClouds() == 1)
                    {
                        // Case: reference is the map just for first iteration (-> visualization)
                        // set AlignedCloud to be next reference
                        aligned_clouds_graph_->updateReference(aligned_clouds_graph_->getNbClouds()-1);
                    }
                }
                else
                {
                    // Case: risk_prediction_(0,0) > class_params_.svm.threshold
                    // rely on prior pose for one step (alignment not performed!)
                    cloud->updateCloud(read_prefiltered, true);
                    // add AlignedCloud to graph
                    aligned_clouds_graph_->addCloud(cloud);
                    aligned_clouds_graph_->updateReference(aligned_clouds_graph_->getNbClouds()-1);
                    updates_counter_ ++;
                    cout << "[Main] -----> ALIGNMENT RISK REFERENCE UPDATE" << endl;
                }

                initialT_ = correction * initialT_;

                // Store chain of corrections for publishing
                total_correction_ = fromMatrix4fToIsometry3d(initialT_);
                updated_correction_ = true;

                // Store aligned map and publish
                if(aligned_clouds_graph_->getLastCloud()->isReference())
                {
                    vis_->publishPose(aligned_clouds_graph_->getCurrentReference()->getCorrectedPose(), 0, "",
                                      cloud->getUtime());
                    reference_vis_ = aligned_clouds_graph_->getCurrentReference()->getCloud();
                    vis_->publishCloud(reference_vis_, 0, "", cloud->getUtime());
                    // Output map
                    aligned_map_ = aligned_map_ + *reference_vis_;
                    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_map_ptr = aligned_map_.makeShared();
                    vis_->publishMap(aligned_map_ptr, cloud->getUtime(), 1);
                }
                else if(cl_cfg_.localize_against_prior_map &&
                        (aligned_clouds_graph_->getNbClouds()-1) % cl_cfg_.reference_update_frequency == 0)
                {
                    vis_->publishPose(aligned_clouds_graph_->getLastCloud()->getCorrectedPose(),
                                      0, "", cloud->getUtime());
                    reference_vis_ = aligned_clouds_graph_->getLastCloud()->getCloud();
                    vis_->publishCloud(reference_vis_, 0, "", cloud->getUtime());
                    // Add last aligned reference to map
                    if(cl_cfg_.merge_aligned_clouds_to_map)
                    {
                        pcl::PointCloud<pcl::PointXYZ>::Ptr merged_map (new pcl::PointCloud<pcl::PointXYZ>);
                        *merged_map = *(prior_map_->getCloud()) + *output;
                        prior_map_->updateCloud(merged_map, 0);
                    }
                }

                // Downsample prior map at very low frequency
                // (once every 30 clouds -> amortized time)
                if(cl_cfg_.localize_against_prior_map && cl_cfg_.merge_aligned_clouds_to_map &&
                   (aligned_clouds_graph_->getNbClouds()-1) % 30 == 0)
                {
                    pcl::PointCloud<pcl::PointXYZ>::Ptr map_prefiltered (new pcl::PointCloud<pcl::PointXYZ>);
                    regionGrowingUniformPlaneSegmentationFilter(prior_map_->getCloud(), map_prefiltered);
                    prior_map_->updateCloud(map_prefiltered, 0);
                }

                if (cl_cfg_.verbose)
                {
                    // Publish aligned reading cloud
                    last_reading_vis_ = aligned_clouds_graph_->getLastCloud()->getCloud();
//                    vis_->publishCloud(last_reading_vis_, 5030, "Aligned Reading"); // TODO: update to unique template LCM - ROS
//                    vis_->publishCloud(last_reading_vis_, 1, "", cloud->getUtime());

                    // Save aligned reading cloud to file
                    stringstream aligned_read;
                    aligned_read << data_directory_path_.str();
                    aligned_read << "/reading_aligned.pcd";
                    pcd_writer_.write<pcl::PointXYZ> (aligned_read.str (), *aligned_clouds_graph_->getLastCloud()->getCloud(), false);
                }

                TimingUtils::toc();

                cout << "============================" << endl
                     << "[Main] Summary:" << endl
                     << "============================" << endl;
                cout << "Reference: " << aligned_clouds_graph_->getLastCloud()->getItsReferenceId() << endl;
                cout << "Reading: " << aligned_clouds_graph_->getLastCloudId() << endl;
                cout << "Number Clouds: " << aligned_clouds_graph_->getNbClouds() << endl;
                cout << "Output Map Size: " << aligned_map_.size() << endl;
                if (cl_cfg_.load_map_from_file || cl_cfg_.localize_against_prior_map)
                    cout << "Prior Map Size: " << prior_map_->getCloud()->size() << endl;
                cout << "Next Reference: " << aligned_clouds_graph_->getCurrentReferenceId() << endl;
                cout << "Updates: " << updates_counter_ << endl;
            }
            cout << "--------------------------------------------------------------------------------------" << endl;
        }
    }
}
} // namespace aicp
