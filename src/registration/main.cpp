// aicp-registration

// Test 3D point clouds alignment
//
// Options: - takes 2 clouds from user and gives
//            transformation between them (-a reference, -b reading)

#include <sstream>  // stringstream
#include <map>
#include <random>

// Project lib
#include "aicpRegistration/registration.hpp"
#include "aicpRegistration/common.hpp"

#include "commonUtils/cloudIO.h"
#include "commonUtils/common.hpp"
#include "drawingUtils/drawingUtils.hpp"

// yaml
#include "yaml-cpp/yaml.h" // read the yaml config

// pcl
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

// args
#include <ConciseArgs>

// lcm
#include <lcm/lcm-cpp.hpp>
#include <boost/shared_ptr.hpp>

using namespace std;
using namespace aicp;

struct CommandLineConfig
{
  string configFile;
  string pointCloudA;
  string pointCloudB;
};

int main(int argc, char **argv)
{
  CommandLineConfig cl_cfg;
  cl_cfg.configFile.append(CONFIG_LOC);
  cl_cfg.configFile.append(PATH_SEPARATOR);
  cl_cfg.configFile.append("aicp_config.yaml");
  cl_cfg.pointCloudA = "";
  cl_cfg.pointCloudB = "";

  ConciseArgs parser(argc, argv, "test-registration");
  parser.add(cl_cfg.configFile, "c", "config_file", "Config file location");
  parser.add(cl_cfg.pointCloudA, "a", "point_cloud_reference", "Pointcloud A");
  parser.add(cl_cfg.pointCloudB, "b", "point_cloud_reading", "Pointcloud B");
  parser.parse();

  /*===================================
  =        Load Input Clouds          =
  ===================================*/
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_A_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_B_ptr (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PointCloud<pcl::PointXYZ>& point_cloud_A = *point_cloud_A_ptr;
  if (cl_cfg.pointCloudA.compare("") != 0) {
    cout << "[Main] Loading point cloud A..." << endl;
    if (pcl::io::loadPCDFile (cl_cfg.pointCloudA, point_cloud_A) == -1) {
      cerr << "Was not able to open file \""<<cl_cfg.pointCloudA<<"\"." << endl;
      return EXIT_SUCCESS;
    }
    cout << "[Main] Point cloud A loaded." << endl;
  }
  else {
    cout << "Please specify point cloud file." << endl;
    return EXIT_SUCCESS;
  }

  pcl::PointCloud<pcl::PointXYZ>& point_cloud_B = *point_cloud_B_ptr;
  if (cl_cfg.pointCloudB.compare("") != 0) {
    cout << "[Main] Loading point cloud B..." << endl;
    if (pcl::io::loadPCDFile (cl_cfg.pointCloudB, point_cloud_B) == -1) {
      cerr << "Was not able to open file \""<<cl_cfg.pointCloudB<<"\"." << endl;
      return EXIT_SUCCESS;
    }
    cout << "[Main] Point cloud B loaded." << endl;
  }
  else {
    cout << "Please specify point cloud file." << endl;
    return EXIT_SUCCESS;
  }

  RegistrationParams params;
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
      params.type = it->second.as<string>();
    }
    else if(key.compare("sensorRange") == 0) {
      params.sensorRange =  it->second.as<float>();
    }
    else if(key.compare("sensorAngularView") == 0) {
      params.sensorAngularView =  it->second.as<float>();
    }
    else if(key.compare("loadPosesFromFile") == 0) {
      params.loadPosesFromFile = it->second.as<string>();
    }
    else if(key.compare("initialTransform") == 0) {
      params.initialTransform = it->second.as<string>();
    }
    else if(key.compare("saveCorrectedPose") == 0) {
      params.saveCorrectedPose =  it->second.as<bool>();
    }
    else if(key.compare("saveInitializedReadingCloud") == 0) {
      params.saveInitializedReadingCloud =  it->second.as<bool>();
    }
    else if(key.compare("saveRegisteredReadingCloud") == 0) {
      params.saveRegisteredReadingCloud =  it->second.as<bool>();
    }
    else if(key.compare("enableLcmVisualization") == 0) {
      params.enableLcmVisualization =  it->second.as<bool>();
    }
  }
  if(params.type.compare("Pointmatcher") == 0) {

    YAML::Node pointmatcherNode = registrationNode["Pointmatcher"];

    for(YAML::const_iterator it=pointmatcherNode.begin();it != pointmatcherNode.end();++it) {
      const string key = it->first.as<string>();

      if(key.compare("configFileName") == 0) {
        params.pointmatcher.configFileName.append(FILTERS_CONFIG_LOC);
        params.pointmatcher.configFileName.append(PATH_SEPARATOR);
        params.pointmatcher.configFileName = FILTERS_CONFIG_LOC + PATH_SEPARATOR + it->second.as<string>();
      }
      else if(key.compare("printOutputStatistics") == 0) {
        params.pointmatcher.printOutputStatistics =  it->second.as<bool>();
      }
    }
  }
  else if(params.type.compare("GICP") == 0) {

    YAML::Node gicpNode = registrationNode["GICP"];

    for(YAML::const_iterator it=gicpNode.begin();it != gicpNode.end();++it) {
      const string key = it->first.as<string>();
      const float val = it->second.as<float>();
    }
  }

  cout << "============================" << endl
       << "Parsed YAML Config" << endl
       << "============================" << endl;

  cout << "[Main] Registration Type: "                 << params.type                          << endl;
  cout << "[Main] Sensor Range: "                      << params.sensorRange                   << endl;
  cout << "[Main] Sensor Angular View: "               << params.sensorAngularView             << endl;
  cout << "[Main] Load Poses from File: "              << params.loadPosesFromFile             << endl;
  cout << "[Main] Initial Transform: "                 << params.initialTransform              << endl;
  cout << "[Main] Save Corrected Pose: "               << params.saveCorrectedPose             << endl;
  cout << "[Main] Save Initialized Reading Cloud: "    << params.saveInitializedReadingCloud   << endl;
  cout << "[Main] Save Registered Reading Cloud: "     << params.saveRegisteredReadingCloud    << endl;
  cout << "[Main] Enable Lcm Visualization: "          << params.enableLcmVisualization        << endl;

  if(params.type.compare("Pointmatcher") == 0) {
    cout << "[Pointmatcher] Config File Name: "                << params.pointmatcher.configFileName        << endl;
    cout << "[Pointmatcher] Print Registration Statistics: "   << params.pointmatcher.printOutputStatistics << endl;
  }
  else if(params.type.compare("GICP") == 0) {
  }
  cout << "============================" << endl;

  /*===================================
  =          Load Input Poses         =
  ===================================*/

  Eigen::Matrix4f ground_truth_reference_pose = Eigen::Matrix4f::Identity(4,4);
  Eigen::Matrix4f ground_truth_reading_pose = Eigen::Matrix4f::Identity(4,4);
  int point_cloud_A_number = -1;
  int point_cloud_B_number = -1;
  if (!params.loadPosesFromFile.empty())
  {
     std::stringstream ssA(extract_ints(cl_cfg.pointCloudA));
     std::istringstream issA(ssA.str());
     issA >> point_cloud_A_number;
     string lineA_from_file = readLineFromFile(params.loadPosesFromFile, point_cloud_A_number);
     ground_truth_reference_pose = parseTransformationQuaternions(lineA_from_file);

     std::stringstream ssB(extract_ints(cl_cfg.pointCloudB));
     std::istringstream issB(ssB.str());
     issB >> point_cloud_B_number;
     string lineB_from_file = readLineFromFile(params.loadPosesFromFile, point_cloud_B_number);
     ground_truth_reading_pose = parseTransformationQuaternions(lineB_from_file);
     cout << "============================" << endl
          << "Ground Truth Reading Pose:" << endl
          << "============================" << endl
          << ground_truth_reading_pose << endl;
  }

  /*===================================
  =      Initialize Reading Pose       =
  ===================================*/

  if (params.initialTransform == "random")
  {
    // random samples from Gaussian distribution with 0 mean and 10 cm variance
    Eigen::VectorXf vars = get_random_gaussian_variable(0, 0.10, 3);

    std::stringstream perturbation;
    perturbation << vars(0);      //x[m]
    perturbation << ',';
    perturbation << vars(1);      //y[m]
    perturbation << ',';
    perturbation << vars(2)*10.0; //yaw[deg]
    params.initialTransform = perturbation.str();
  }

  cout << "[Main] Initialization: " << params.initialTransform << endl;
  Eigen::Matrix4f perturbation = parseTransformationDeg(params.initialTransform);
  Eigen::Matrix4f estimated_reading_pose = perturbation * ground_truth_reading_pose;

  /*===================================
  =     Initialize Reading Cloud      =
  ===================================*/

  pcl::PointCloud<pcl::PointXYZ>::Ptr initialized_reading_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::transformPointCloud (*point_cloud_B_ptr, *initialized_reading_cloud_ptr, perturbation);

  /*===================================
  =        Filter Input Clouds        =
  ===================================*/

  pcl::PointCloud<pcl::PointXYZ> overlap_points_A;
  pcl::PointCloud<pcl::PointXYZ> overlap_points_B;
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudA_matched_planes (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudB_matched_planes (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr eigenvectors (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

  // ------------------
  // FOV-based Overlap
  // ------------------
  Eigen::Isometry3d ground_truth_reference_pose_iso = fromMatrix4fToIsometry3d(ground_truth_reference_pose);
  Eigen::Isometry3d estimated_reading_pose_iso = fromMatrix4fToIsometry3d(estimated_reading_pose);
  float fov_overlap = overlapFilter(point_cloud_A, *initialized_reading_cloud_ptr,
                                    ground_truth_reference_pose_iso, estimated_reading_pose_iso,
                                    params.sensorRange , params.sensorAngularView,
                                    overlap_points_A, overlap_points_B);
  cout << "==============================" << endl
       << "FOV-based Overlap: " << fov_overlap << " %" << endl
       << "==============================" << endl;

  // ------------------------------------
  // Pre-filtering: 1) down-sampling
  //                2) planes extraction
  // ------------------------------------
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_A_prefiltered (new pcl::PointCloud<pcl::PointXYZ>);
  regionGrowingUniformPlaneSegmentationFilter(point_cloud_A_ptr, point_cloud_A_prefiltered);
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_B_prefiltered (new pcl::PointCloud<pcl::PointXYZ>);
  regionGrowingUniformPlaneSegmentationFilter(initialized_reading_cloud_ptr, point_cloud_B_prefiltered);

  // ---------------------
  // Octree-based Overlap
  // ---------------------

  // -------------
  // Alignability
  // -------------
  // Alignability computed on points belonging to the region of overlap (overlap_points_A, overlap_points_B)
  float alignability = alignabilityFilter(overlap_points_A, overlap_points_B,
                                          ground_truth_reference_pose_iso, estimated_reading_pose_iso,
                                          cloudA_matched_planes, cloudB_matched_planes, eigenvectors);
  cout << "[Main] Alignability (degenerate if ~ 0): " << alignability << endl;

  /*===================================
  =              AICP Core            =
  ===================================*/
  string configNameAICP;
  configNameAICP.append(FILTERS_CONFIG_LOC);
  configNameAICP.append("/icp_autotuned.yaml");

  // Auto-tune ICP chain (quantile for the outlier filter)
  float current_ratio = fov_overlap/100.0;
  if (current_ratio < 0.25)
    current_ratio = 0.25;
  else if (current_ratio > 0.70)
    current_ratio = 0.70;

  replaceRatioConfigFile(params.pointmatcher.configFileName, configNameAICP, current_ratio);
//  params.pointmatcher.configFileName = configNameAICP;

  /*===================================
  =          Register Clouds          =
  ===================================*/

  Eigen::Matrix4f T = Eigen::Matrix4f::Zero(4,4);

  std::unique_ptr<AbstractRegistrator> registration = create_registrator(params);
//  registration->registerClouds(*point_cloud_A_prefiltered, *point_cloud_B_prefiltered, T);
  registration->registerClouds(point_cloud_A, *initialized_reading_cloud_ptr, T);

  cout << "============================" << endl
       << "Computed 3D Transform:" << endl
       << "============================" << endl
       << T << endl;

  Eigen::Matrix4f corrected_reading_pose;
  corrected_reading_pose = T * estimated_reading_pose;
  cout << "============================" << endl
       << "Corrected Reading Pose:" << endl
       << "============================" << endl
       << corrected_reading_pose << endl;

  /*===================================
  =            Save Clouds            =
  ===================================*/
  pcl::PCDWriter writer;
  if (params.saveInitializedReadingCloud) {
    stringstream ss;
    ss << "initialized_reading_cloud.pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *initialized_reading_cloud_ptr, false);
  }
  if (params.saveRegisteredReadingCloud) {
    pcl::PointCloud<pcl::PointXYZ> registered_reading_cloud;
    registration->getOutputReading(registered_reading_cloud);
    stringstream ss;
    ss << "registered_reading_cloud.pcd";
    writer.write<pcl::PointXYZ> (ss.str (), registered_reading_cloud, false);
  }

  if ((point_cloud_A_number != -1) && params.saveCorrectedPose) {
    stringstream ss;
    ss << "corrected_poses/corrected_pose_";
    ss << point_cloud_A_number;
    ss << "_";
    ss << point_cloud_B_number;
    ss << ".txt";
    write3DTransformToFile(corrected_reading_pose, ss.str(), point_cloud_A_number, point_cloud_B_number);
  }

  if (params.enableLcmVisualization) {
    boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
    if(!lcm->good()) {
      std::cerr << "[Main] LCM is not good for visualization." << std::endl;
    }
    Eigen::Isometry3d global_reference_frame = Eigen::Isometry3d::Identity();

    /*===================================
    =          Visualize Poses          =
    ===================================*/
    pcl::PointCloud<pcl::PointXYZRGBNormal> cloud_estimated_pose;
    pcl::PointCloud<pcl::PointXYZRGBNormal> cloud_corrected_pose;
    pcl::PointCloud<pcl::PointXYZRGBNormal> cloud_ground_truth_pose;

    // Fill in the estimated pose
    cloud_estimated_pose.width    = 4;
    cloud_estimated_pose.height   = 1;
    cloud_estimated_pose.is_dense = false;
    cloud_estimated_pose.points.resize (cloud_estimated_pose.width * cloud_estimated_pose.height);

    for (int i = 0; i < 3; i++)
    {
      cloud_estimated_pose.points[i].x = estimated_reading_pose(0,3);
      cloud_estimated_pose.points[i].y = estimated_reading_pose(1,3);
      cloud_estimated_pose.points[i].z = estimated_reading_pose(2,3);
      cloud_estimated_pose.points[i].normal_x = estimated_reading_pose(0,i);
      cloud_estimated_pose.points[i].normal_y = estimated_reading_pose(1,i);
      cloud_estimated_pose.points[i].normal_z = estimated_reading_pose(2,i);
      cloud_estimated_pose.points[i].r = 255.0;
      cloud_estimated_pose.points[i].g = 255.0;
      cloud_estimated_pose.points[i].b = 255.0;
      if (i == 0)
        cloud_estimated_pose.points[i].r = 0.0;
      else if (i == 1)
        cloud_estimated_pose.points[i].g = 0.0;
      else if (i == 2)
        cloud_estimated_pose.points[i].b = 0.0;
    }
    drawPointCloudNormalsCollections(lcm, 1, global_reference_frame, cloud_estimated_pose, 0, "Estimated Pose B");

    // Fill in the corrected pose
    cloud_corrected_pose.width    = 4;
    cloud_corrected_pose.height   = 1;
    cloud_corrected_pose.is_dense = false;
    cloud_corrected_pose.points.resize (cloud_corrected_pose.width * cloud_corrected_pose.height);

    for (int i = 0; i < 3; i++)
    {
      cloud_corrected_pose.points[i].x = corrected_reading_pose(0,3);
      cloud_corrected_pose.points[i].y = corrected_reading_pose(1,3);
      cloud_corrected_pose.points[i].z = corrected_reading_pose(2,3);
      cloud_corrected_pose.points[i].normal_x = corrected_reading_pose(0,i);
      cloud_corrected_pose.points[i].normal_y = corrected_reading_pose(1,i);
      cloud_corrected_pose.points[i].normal_z = corrected_reading_pose(2,i);
      cloud_corrected_pose.points[i].r = 255.0;
      cloud_corrected_pose.points[i].g = 255.0;
      cloud_corrected_pose.points[i].b = 255.0;
      if (i == 0)
        cloud_corrected_pose.points[i].r = 150.0;
      else if (i == 1)
        cloud_corrected_pose.points[i].g = 150.0;
      else if (i == 2)
        cloud_corrected_pose.points[i].b = 150.0;
    }
    drawPointCloudNormalsCollections(lcm, 3, global_reference_frame, cloud_corrected_pose, 0, "Corrected Pose B");

    // Fill in the ground truth pose
    cloud_ground_truth_pose.width    = 4;
    cloud_ground_truth_pose.height   = 1;
    cloud_ground_truth_pose.is_dense = false;
    cloud_ground_truth_pose.points.resize (cloud_ground_truth_pose.width * cloud_ground_truth_pose.height);

    for (int i = 0; i < 3; i++)
    {
      cloud_ground_truth_pose.points[i].x = ground_truth_reading_pose(0,3);
      cloud_ground_truth_pose.points[i].y = ground_truth_reading_pose(1,3);
      cloud_ground_truth_pose.points[i].z = ground_truth_reading_pose(2,3);
      cloud_ground_truth_pose.points[i].normal_x = ground_truth_reading_pose(0,i);
      cloud_ground_truth_pose.points[i].normal_y = ground_truth_reading_pose(1,i);
      cloud_ground_truth_pose.points[i].normal_z = ground_truth_reading_pose(2,i);
      cloud_ground_truth_pose.points[i].r = 0.0;
      cloud_ground_truth_pose.points[i].g = 0.0;
      cloud_ground_truth_pose.points[i].b = 0.0;
      if (i == 0)
        cloud_ground_truth_pose.points[i].r = 255.0;
      else if (i == 1)
        cloud_ground_truth_pose.points[i].g = 255.0;
      else if (i == 2)
        cloud_ground_truth_pose.points[i].b = 255.0;
    }
    drawPointCloudNormalsCollections(lcm, 5, global_reference_frame, cloud_ground_truth_pose, 0, "Ground Truth Pose B");

    /*===================================
    =      Visualize Alignability       =
    ===================================*/
    drawPointCloudNormalsCollections(lcm, 9, global_reference_frame, *cloudA_matched_planes, 0, "Matches A");
    drawPointCloudNormalsCollections(lcm, 11, global_reference_frame, *cloudB_matched_planes, 0, "Matches B");
    eigenvectors->points.resize(4);
    drawPointCloudNormalsCollections(lcm, 13, global_reference_frame, *eigenvectors, 0, "Alignability Eigenvectors");
  }

  return 0;
}
