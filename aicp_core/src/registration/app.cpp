#include "aicp_registration/app.hpp"
#include "aicp_registration/registration.hpp"
#include "aicp_overlap/overlap.hpp"
#include "aicp_classification/classification.hpp"

#include "aicp_utils/timing.hpp"
#include "aicp_utils/common.hpp"

namespace aicp {

App::App(const CommandLineConfig& cl_cfg,
         RegistrationParams reg_params,
         OverlapParams overlap_params,
         ClassificationParams class_params) :
    cl_cfg_(cl_cfg), reg_params_(reg_params),
    overlap_params_(overlap_params), class_params_(class_params)
{
    // Create debug data folder
    data_directory_path_ << "/tmp/aicp_data";
    const char* path = data_directory_path_.str().c_str();
    boost::filesystem::path dir(path);
    if(boost::filesystem::exists(path))
        boost::filesystem::remove_all(path);
    if(boost::filesystem::create_directory(dir))
    {
        cout << "Create AICP debug data directory: " << path << endl
             << "============================" << endl;
    }

    // Instantiate objects
    registr_ = create_registrator(reg_params_);
    overlapper_ = create_overlapper(overlap_params_);
    classifier_ = create_classifier(class_params_);
}

void App::setReference(AlignedCloudPtr& reading_cloud,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr& reference_cloud,
                       Eigen::Isometry3d& reference_pose)
{
    // Crop map
    pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_map (new pcl::PointCloud<pcl::PointXYZ>);
    if (cl_cfg_.load_map_from_file || cl_cfg_.localize_against_prior_map)
    {
        // crop map around current reading pose
        *cropped_map = *(prior_map_->getCloud());
        Eigen::Matrix4f tmp = (reading_cloud->getPriorPose()).matrix().cast<float>();
        getPointsInOrientedBox(cropped_map,
                               -cl_cfg_.crop_map_around_base,
                               cl_cfg_.crop_map_around_base, tmp);
    }

    // Set reference cloud
    if (!first_cloud_initialized_ || cl_cfg_.localize_against_prior_map)
    {
        reference_cloud = cropped_map;
        reference_pose = reading_cloud->getPriorPose();
        first_cloud_initialized_ = true;
    }
    else if (cl_cfg_.localize_against_built_map)
    {
        copyPointCloud(aligned_map_, *cropped_map);
        Eigen::Matrix4f tmp = (reading_cloud->getPriorPose()).matrix().cast<float>();
        getPointsInOrientedBox(cropped_map,
                               -cl_cfg_.crop_map_around_base,
                               cl_cfg_.crop_map_around_base, tmp);
        reference_cloud = cropped_map;
        reference_pose = reading_cloud->getPriorPose();
    }
    else
    {
        reference_cloud = aligned_clouds_graph_->getCurrentReference()->getCloud();
        reference_pose = aligned_clouds_graph_->getCurrentReference()->getCorrectedPose();
    }
}

void App::setAndFilterReading(AlignedCloudPtr& reading_cloud_in,
                              pcl::PointCloud<pcl::PointXYZ>::Ptr& reading_cloud_out,
                              Eigen::Isometry3d& reading_pose)
{
    reading_pose = reading_cloud_in->getPriorPose();

    // Initialize cloud before sending to filters
    // (simulates correction integration only if "debug" mode
    // and integrates interactive marker pose)
    pcl::PointCloud<pcl::PointXYZ>::Ptr reading_tmp (new pcl::PointCloud<pcl::PointXYZ>);
    if (cl_cfg_.working_mode == "robot")
        *reading_tmp = *(reading_cloud_in->getCloud());
    else
    {
        pcl::transformPointCloud (*(reading_cloud_in->getCloud()), *reading_tmp, initialT_);
        Eigen::Isometry3d initialT_iso = fromMatrix4fToIsometry3d(initialT_);
        reading_pose = initialT_iso * reading_pose;
        // Update AlignedCloud pose
        reading_cloud_in->setPriorPose(reading_pose);
    }

    // Pre-filter reading cloud
    filterCloud(reading_tmp, reading_cloud_out);
}

void App::filterCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out)
{
    // ------------------------------------
    // Pre-filtering: 1) down-sampling
    //                2) planes extraction
    // ------------------------------------
    regionGrowingUniformPlaneSegmentationFilter(cloud_in, cloud_out);
}

void App::computeOverlap(pcl::PointCloud<pcl::PointXYZ>::Ptr& reference_cloud,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr& reading_cloud,
                         Eigen::Isometry3d& reference_pose,
                         Eigen::Isometry3d& reading_pose)
{
    // ---------------------
    // Octree-based Overlap
    // ---------------------
    ColorOcTree* ref_tree;
    ColorOcTree* read_tree = new ColorOcTree(overlap_params_.octree_based.octomapResolution);

    if(//(cl_cfg_.load_map_from_file && aligned_clouds_graph_->getNbClouds() == 0) ||
        cl_cfg_.localize_against_prior_map)
    {
        octree_overlap_ = 50.0;
    }
    else
    {
        // 1) create octree from reference cloud (wrt robot's point of view)
        // 2) add the reading cloud and compute overlap
        ref_tree = overlapper_->computeOverlap(*reference_cloud, *reading_cloud,
                                               reference_pose, reading_pose,
                                               read_tree);
        octree_overlap_ = overlapper_->getOverlap();
    }

    cout << "====================================" << endl
         << "[Main] Octree-based Overlap: " << octree_overlap_ << " %" << endl
         << "====================================" << endl;
}

void App::computeAlignmentRisk(pcl::PointCloud<pcl::PointXYZ>::Ptr& reference_cloud,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr& reading_cloud,
                               Eigen::Isometry3d& reference_pose,
                               Eigen::Isometry3d& reading_pose)
{
    pcl::PointCloud<pcl::PointXYZ> overlap_reference;
    pcl::PointCloud<pcl::PointXYZ> overlap_reading;
    // ------------------
    // FOV-based Overlap
    // ------------------
    fov_overlap_ = overlapFilter(*reference_cloud, *reading_cloud,
                                 reference_pose, reading_pose,
                                 reg_params_.sensorRange , reg_params_.sensorAngularView,
                                 overlap_reference, overlap_reading);

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr matched_planes_reference (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr matched_planes_reading (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr eigenvectors (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    // -------------
    // Alignability
    // -------------
    // Alignability computed on points belonging to the region of overlap (overlap_points_A, overlap_points_B)
    alignability_ = alignabilityFilter(overlap_reference, overlap_reading,
                                       reference_pose, reading_pose,
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
}

void App::computeRegistration(pcl::PointCloud<pcl::PointXYZ>& reference,
                              pcl::PointCloud<pcl::PointXYZ>& reading,
                              Eigen::Matrix4f &T)
{
    /*===================================
    =              AICP Core            =
    ===================================*/
    string configNameAICP;
    configNameAICP.append(cl_cfg_.registration_config_file);

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

void App::runAicpPipeline(pcl::PointCloud<pcl::PointXYZ>::Ptr& reference_prefiltered,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr& reading_prefiltered,
                          Eigen::Isometry3d& reference_pose,
                          Eigen::Isometry3d& reading_pose,
                          Eigen::Matrix4f &T)
{
    /*==========================
    =          Overlap         =
    ===========================*/

    computeOverlap(reference_prefiltered, reading_prefiltered, reference_pose, reading_pose);

    /*=================================
    =          Alignment Risk         =
    =================================*/

    if (cl_cfg_.failure_prediction_mode)
        computeAlignmentRisk(reference_prefiltered, reading_prefiltered, reference_pose, reading_pose);

    /*================================
    =          Registration          =
    ================================*/
    if(!cl_cfg_.failure_prediction_mode ||                      // if alignment risk disabled
       risk_prediction_(0,0) <= class_params_.svm.threshold)    // or below threshold
        computeRegistration(*reference_prefiltered, *reading_prefiltered, T);
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
                // Update AlignedCloud
                cloud->updateCloud(ref_prefiltered, true);
                // Initialize graph
                aligned_clouds_graph_->initialize(cloud);

                // VISUALIZE first reference cloud
                reference_vis_ = aligned_clouds_graph_->getCurrentReference()->getCloud();
                vis_->publishCloud(reference_vis_, 0, "", cloud->getUtime());

                // Path
                vis_->publishPoses(aligned_clouds_graph_->getCurrentReference()->getCorrectedPose(), 0, "",
                                   cloud->getUtime());

                // Store built map
                aligned_map_ = aligned_map_ + *reference_vis_;
                // VISUALIZE built map
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
                }

                /*========================
                =          Input         =
                ========================*/

                Eigen::Isometry3d ref_pose;
                pcl::PointCloud<pcl::PointXYZ>::Ptr ref_prefiltered;
                setReference(cloud, ref_prefiltered, ref_pose);

                Eigen::Isometry3d read_pose;
                pcl::PointCloud<pcl::PointXYZ>::Ptr read_prefiltered (new pcl::PointCloud<pcl::PointXYZ>);
                setAndFilterReading(cloud, read_prefiltered, read_pose);

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

                /*=====================================
                =          AICP Registration          =
                =====================================*/
                Eigen::Matrix4f correction = Eigen::Matrix4f::Identity(4,4);
                pcl::PointCloud<pcl::PointXYZ>::Ptr output (new pcl::PointCloud<pcl::PointXYZ>);

                runAicpPipeline(ref_prefiltered, read_prefiltered, ref_pose, read_pose, correction);

                if(!cl_cfg_.failure_prediction_mode ||                      // if alignment risk disabled
                   risk_prediction_(0,0) <= class_params_.svm.threshold)    // or below threshold
                {
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
                    // Update AlignedCloud with corrected pose and (prefiltered) cloud after alignment
                    cloud->updateCloud(output, correction_iso, false, aligned_clouds_graph_->getCurrentReferenceId());
                    // Add AlignedCloud to graph
                    aligned_clouds_graph_->addCloud(cloud);

                    // Windowed reference update policy (count number of clouds after last reference)
                    if(((aligned_clouds_graph_->getNbClouds() - (aligned_clouds_graph_->getCurrentReferenceId()+1))
                        % cl_cfg_.reference_update_frequency == 0) &&
                        !cl_cfg_.localize_against_prior_map)
                    {
                        // Set AlignedCloud to be next reference
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

                /*======================================
                =          Save and Visualize          =
                ======================================*/

                // Store chain of corrections for publishing
                total_correction_ = fromMatrix4fToIsometry3d(initialT_);
                updated_correction_ = true;

                // Path (save and visualize)
                // Ensure robot moves between stored poses
                Eigen::Isometry3d relative_motion = vis_->getPath().back().inverse() *
                                                    aligned_clouds_graph_->getLastCloud()->getCorrectedPose();
                double dist = relative_motion.translation().norm();
                if (dist > 1.0)
                {
                    vis_->publishPoses(aligned_clouds_graph_->getLastCloud()->getCorrectedPose(), 0, "",
                                       cloud->getUtime());
                    vis_->publishPriorPoses(aligned_clouds_graph_->getLastCloud()->getPriorPose(), 0, "",
                                       cloud->getUtime());
                    vis_->publishOdomPoses(aligned_clouds_graph_->getLastCloud()->getOdomPose(), 0, "",
                                       cloud->getUtime());


                    std::cout << "odom_to_map publish <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n";
                    Eigen::Isometry3d base_to_odom = aligned_clouds_graph_->getLastCloud()->getOdomPose();
                    Eigen::Isometry3d base_to_map = aligned_clouds_graph_->getLastCloud()->getCorrectedPose();

                    Eigen::Isometry3d odom_to_map = (base_to_map.inverse() * base_to_odom).inverse();
                    std::cout << odom_to_map.translation().transpose() << " odom_to_map publish\n";
                    vis_->publishOdomToMapPose(odom_to_map, cloud->getUtime());

                }

                // Store aligned map and VISUALIZE
                if(aligned_clouds_graph_->getLastCloud()->isReference())
                {
                    // vis_->publishPoses(aligned_clouds_graph_->getCurrentReference()->getCorrectedPose(), 0, "",
                    //                    cloud->getUtime());
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
                    vis_->publishPoses(aligned_clouds_graph_->getLastCloud()->getCorrectedPose(),
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
