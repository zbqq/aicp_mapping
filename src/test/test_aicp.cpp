// Demo program for testing AICP:
//  1. Reading two point clouds from file (with sensor origin information)
//  2. Running overlap analysis
//  3. Running alignability analysis
//  4  Running risk of alignment failure analysis (pre-trained SVM)
//  4. Running AICP alignment
//  5. Writing result to file and console
//  6. Compare file with expected result

// Test data at: <HOME>/drs-testing-data/aicp-data
// Run: rosrun aicp aicp-test

// Get path to AICP base
#ifdef CONFDIR
  # define AICP_BASE CONFDIR
#else
  # define AICP_BASE "undefined"
#endif

#include <iostream>

// Project lib
#include "aicp_registration/registration.hpp"
#include "aicp_registration/common.hpp"

#include "aicp_overlap/overlap.hpp"
#include "aicp_overlap/common.hpp"

#include "aicp_classification/classification.hpp"
#include "aicp_classification/common.hpp"

#include "aicp_common_utils/cloudIO.h"
#include "aicp_common_utils/common.hpp"
#include "aicp_drawing_utils/drawingUtils.hpp"

// yaml
#include "yaml-cpp/yaml.h" // read the yaml config

// pcl
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

using namespace std;
using namespace aicp;

void getPoseFromPointCloud(pcl::PointCloud<pcl::PointXYZRGB>& cloud, Eigen::Isometry3d& pose);
void writeResultToFile(string out_file, int line_number, float filtered_cloud_size, 
                      float fov_overlap, float octree_overlap, float alignability, 
                      Eigen::MatrixXd &risk_prediction, Eigen::Matrix4f &T);
float compareResultFiles(string file1, string file2);

int main (int argc, char** argv)
{
  const char *homedir;
  if ((homedir = getenv("HOME")) == NULL) {
    homedir = getpwuid(getuid())->pw_dir;
  }

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  int number_test_clouds = 16;

  // Overlap parameters
  float fov_overlap_ = -1.0;
  float octree_overlap_ = -1.0;
  // Alignability
  float alignability_ = -1.0;
  // Alignment Risk
  Eigen::MatrixXd risk_prediction_; // ours
  vector<float> other_predictions_; // state of the art

  // Project Objects
  RegistrationParams registration_params_;
  OverlapParams overlap_params_;
  ClassificationParams classification_params_;
  std::unique_ptr<AbstractRegistrator> registr_;
  std::unique_ptr<AbstractOverlapper> overlapper_;
  std::unique_ptr<AbstractClassification> classifier_;

  // Config file
  string config_file;
  config_file.append(CONFIG_LOC);
  config_file.append(PATH_SEPARATOR);
  config_file.append("aicp_test_config.yaml");

  // Expected result file
  std::stringstream expected_file;
  expected_file << homedir;
  expected_file << "/drs-testing-data/aicp-data/test_aicp_expected.txt";
  // Output file
  std::stringstream out_file;
  out_file << homedir;
  out_file << "/drs-testing-data/aicp-data/test_aicp_output.txt";
  std::ofstream ofs;
  ofs.open(out_file.str().c_str(), std::ofstream::out | std::ofstream::trunc);
  ofs.close();

  /*===================================
  =            YAML Config            =
  ===================================*/
  string yamlConfig_;
  YAML::Node yn_;
  yamlConfig_ = config_file;
  yn_ = YAML::LoadFile(yamlConfig_);

  YAML::Node registrationNode = yn_["AICP"]["Registration"];
  for(YAML::const_iterator it=registrationNode.begin();it != registrationNode.end();++it) {

    const string key = it->first.as<string>();

    if(key.compare("type") == 0) {
      registration_params_.type = it->second.as<string>();
    }
    else if(key.compare("sensorRange") == 0) {
      registration_params_.sensorRange =  it->second.as<float>();
    }
    else if(key.compare("sensorAngularView") == 0) {
      registration_params_.sensorAngularView =  it->second.as<float>();
    }
  }
  if(registration_params_.type.compare("Pointmatcher") == 0) {

    YAML::Node pointmatcherNode = registrationNode["Pointmatcher"];

    for(YAML::const_iterator it=pointmatcherNode.begin();it != pointmatcherNode.end();++it) {
      const string key = it->first.as<string>();

      if(key.compare("configFileName") == 0) {
        registration_params_.pointmatcher.configFileName.append(FILTERS_CONFIG_LOC);
        registration_params_.pointmatcher.configFileName.append(PATH_SEPARATOR);
        registration_params_.pointmatcher.configFileName = FILTERS_CONFIG_LOC + PATH_SEPARATOR + it->second.as<string>();
      }
    }
  }
  YAML::Node overlapNode = yn_["AICP"]["Overlap"];
  for(YAML::const_iterator it=overlapNode.begin();it != overlapNode.end();++it) {

    const string key = it->first.as<string>();

    if(key.compare("type") == 0) {
      overlap_params_.type = it->second.as<string>();
    }
  }
  if(overlap_params_.type.compare("OctreeBased") == 0) {

    YAML::Node octreeBasedNode = overlapNode["OctreeBased"];

    for(YAML::const_iterator it=octreeBasedNode.begin();it != octreeBasedNode.end();++it) {
      const string key = it->first.as<string>();

      if(key.compare("octomapResolution") == 0) {
        overlap_params_.octree_based.octomapResolution = it->second.as<float>();
      }
    }
  }
  YAML::Node classificationNode = yn_["AICP"]["Classifier"];
  for (YAML::const_iterator it = classificationNode.begin(); it != classificationNode.end(); ++it) {
    const std::string key = it->first.as<std::string>();

    if (key.compare("type") == 0) {
      classification_params_.type = it->second.as<std::string>();
    }
  }

  if (classification_params_.type.compare("SVM") == 0) {

    YAML::Node svmNode = classificationNode["SVM"];

    for(YAML::const_iterator it=svmNode.begin();it != svmNode.end();++it) {
      const std::string key = it->first.as<std::string>();

      if(key.compare("threshold") == 0) {
        classification_params_.svm.threshold = it->second.as<double>();
      }
      else if(key.compare("trainingFile") == 0) {
        classification_params_.svm.trainingFile = expandEnvironmentVariables(it->second.as<std::string>());
      }
      else if(key.compare("testingFile") == 0) {
          classification_params_.svm.testingFile = expandEnvironmentVariables(it->second.as<std::string>());
      }
      else if(key.compare("saveFile") == 0) {
        classification_params_.svm.saveFile = expandEnvironmentVariables(it->second.as<std::string>());
      }
      else if(key.compare("saveProbs") == 0) {
        classification_params_.svm.saveProbs = expandEnvironmentVariables(it->second.as<std::string>());
      }
      else if(key.compare("modelLocation") == 0) {
        classification_params_.svm.modelLocation = expandEnvironmentVariables(it->second.as<std::string>());
      }
    }
  }

  cout << "============================" << endl
       << "Parsed YAML Config" << endl
       << "============================" << endl;

  cout << "[Main] Registration Type: "                 << registration_params_.type                          << endl;
  cout << "[Main] Sensor Range: "                      << registration_params_.sensorRange                   << endl;
  cout << "[Main] Sensor Angular View: "               << registration_params_.sensorAngularView             << endl;

  if(registration_params_.type.compare("Pointmatcher") == 0) {
    cout << "[Pointmatcher] Config File Name: "                << registration_params_.pointmatcher.configFileName        << endl;
  }

  cout << "[Main] Overlap Type: "                   << overlap_params_.type                             << endl;

  if(overlap_params_.type.compare("OctreeBased") == 0) {
    cout << "[OctreeBased] Octomap Resolution: "    << overlap_params_.octree_based.octomapResolution   << endl;
  }

  cout << "[Main] Classification Type: "       << classification_params_.type                    << endl;

  if(classification_params_.type.compare("SVM") == 0) {
    cout << "[SVM] Acceptance Threshold: "    << classification_params_.svm.threshold           << endl;
    cout << "[SVM] Training File: "           << classification_params_.svm.trainingFile        << endl;
    cout << "[SVM] Testing File: "            << classification_params_.svm.testingFile         << endl;
    cout << "[SVM] Saving Model To: "         << classification_params_.svm.saveFile            << endl;
    cout << "[SVM] Saving Probs To: "         << classification_params_.svm.saveProbs           << endl;
    cout << "[SVM] Loading Model From: "      << classification_params_.svm.modelLocation       << endl;
  }

  cout << "============================" << endl;

  // Instantiate objects
  registr_ = create_registrator(registration_params_);
  overlapper_ = create_overlapper(overlap_params_);
  classifier_ = create_classifier(classification_params_);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr ref_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  Eigen::Isometry3d reference_pose;

  for (int i = 0; i < number_test_clouds; i ++)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr read_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    Eigen::Isometry3d reading_pose;
  
    std::stringstream pcl_name;
    pcl_name << homedir;
    pcl_name << "/drs-testing-data/aicp-data/cloud_";
    pcl_name << std::to_string(i);
    pcl_name << ".pcd";

    if (i == 0) // load reference cloud from file
    {
      /*===================================
      =   Load Reference Cloud and Pose   =
      ===================================*/
      if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (pcl_name.str().c_str(), *ref_ptr) == -1)
      {
        PCL_ERROR ("Error loading PCD file.\n");
        return (-1);
      }
      std::cout << "Loaded " << ref_ptr->width * ref_ptr->height << " data points from " << pcl_name.str().c_str() << std::endl;

      getPoseFromPointCloud(*ref_ptr, reference_pose);
    }
    else // load i-th reading cloud from file
    {
      /*===================================
      =    Load Reading Cloud and Pose    =
      ===================================*/      
      if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (pcl_name.str().c_str(), *read_ptr) == -1)
      {
        PCL_ERROR ("Error loading PCD file.\n");
        return (-1);
      }      
      std::cout << "Loaded " << read_ptr->width * read_ptr->height << " data points from " << pcl_name.str().c_str() << std::endl;

      getPoseFromPointCloud(*read_ptr, reading_pose);

      /*===================================
      =        Filter Input Clouds        =
      ===================================*/

      pcl::PointCloud<pcl::PointXYZ>::Ptr ref_xyz_ptr (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr read_xyz_ptr (new pcl::PointCloud<pcl::PointXYZ>);
      copyPointCloud(*ref_ptr, *ref_xyz_ptr);
      copyPointCloud(*read_ptr, *read_xyz_ptr);

      pcl::PointCloud<pcl::PointXYZ> overlap_points_ref;
      pcl::PointCloud<pcl::PointXYZ> overlap_points_read;
      pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr ref_matched_planes (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
      pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr read_matched_planes (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
      pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr eigenvectors (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

      // ------------------
      // FOV-based Overlap
      // ------------------
      fov_overlap_ = overlapFilter(*ref_xyz_ptr, *read_xyz_ptr,
                                   reference_pose, reading_pose,
                                   registration_params_.sensorRange , registration_params_.sensorAngularView,
                                   overlap_points_ref, overlap_points_read);
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

      // ---------------------
      // Octree-based Overlap
      // ---------------------
      ColorOcTree* ref_tree;
      ColorOcTree* read_tree = new ColorOcTree(overlap_params_.octree_based.octomapResolution);

      // Create octree from reference cloud (wrt robot point of view),
      // add the reading cloud and compute overlap
      ref_tree = overlapper_->computeOverlap(*ref_xyz_prefiltered, *read_xyz_prefiltered,
                                             reference_pose, reading_pose,
                                             read_tree);
      octree_overlap_ = overlapper_->getOverlap();

      cout << "====================================" << endl
           << "[Main] Octree-based Overlap: " << octree_overlap_ << " %" << endl
           << "====================================" << endl;

      // -------------
      // Alignability
      // -------------
      // Alignability computed on points belonging to the region of overlap (overlap_points_A, overlap_points_B)
      alignability_ = alignabilityFilter(overlap_points_ref, overlap_points_read,
                                         reference_pose, reading_pose,
                                         ref_matched_planes, read_matched_planes, eigenvectors);
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

      replaceRatioConfigFile(registration_params_.pointmatcher.configFileName, configNameAICP, current_ratio);
      registr_->updateConfigParams(configNameAICP);
      //registration_params_.pointmatcher.configFileName = configNameAICP;

      /*===================================
      =          Register Clouds          =
      ===================================*/

      Eigen::Matrix4f T = Eigen::Matrix4f::Zero(4,4);

      vector<float> soa_predictions;
      registr_->registerClouds(*ref_xyz_prefiltered, *read_xyz_prefiltered, T, soa_predictions);
      if (!soa_predictions.empty())
      {
        cout << "[Main] Degeneracy (degenerate if ~ 0): " << soa_predictions.at(0) << " %" << endl;
        cout << "[Main] ICN (degenerate if ~ 0): " << soa_predictions.at(1) << endl;
      }

      cout << "============================" << endl
           << "Computed 3D Transform:" << endl
           << "============================" << endl
           << T << endl;

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::transformPointCloud (*read_ptr, *out_ptr, T);

      // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      // // Publish clouds and octrees to LCM
      // Eigen::Isometry3d local = Eigen::Isometry3d::Identity();
      // drawPointCloudCollections(lcm, 0, local, *ref_ptr, 1, "Reference");
      // drawPointCloudCollections(lcm, 1, local, *read_ptr, 1, "Reading");
      // drawPointCloudCollections(lcm, 2, local, *out_ptr, 1, "Output");
      // publishOctreeToLCM(lcm, ref_tree, "OCTOMAP_REF");
      // publishOctreeToLCM(lcm, read_tree, "OCTOMAP");
      // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      // Write results to file
      writeResultToFile(out_file.str().c_str(), i, read_xyz_prefiltered->size(),
                        fov_overlap_, octree_overlap_, alignability_, 
                        risk_prediction_, T);
      // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      cout << "\n" << "-------------------------------------------------------------------------------" << "\n \n";
    }
  }
  // Compare with expected results from file
  float match = compareResultFiles(expected_file.str().c_str(), out_file.str().c_str());
  cout << "Results Match: " << match << "%" << endl;

  return (0);
}

void getPoseFromPointCloud(pcl::PointCloud<pcl::PointXYZRGB>& cloud, Eigen::Isometry3d& pose)
{
  pose.setIdentity();
  pose.translation() << cloud.sensor_origin_.x(), cloud.sensor_origin_.y(), cloud.sensor_origin_.z();
  Eigen::Quaterniond quat = Eigen::Quaterniond(cloud.sensor_orientation_.w(),
                                               cloud.sensor_orientation_.x(),
                                               cloud.sensor_orientation_.y(),
                                               cloud.sensor_orientation_.z());
  pose.rotate(quat);
}

void writeResultToFile(string out_file, int line_number, float filtered_cloud_size, 
                      float fov_overlap, float octree_overlap, float alignability, 
                      Eigen::MatrixXd &risk_prediction, Eigen::Matrix4f &T)
{
  Eigen::MatrixXf line = Eigen::MatrixXf::Zero(1, 21);
  line << filtered_cloud_size, fov_overlap, octree_overlap, alignability, risk_prediction(0,0),
          T(0,0), T(0,1), T(0,2), T(0,3),
          T(1,0), T(1,1), T(1,2), T(1,3),
          T(2,0), T(2,1), T(2,2), T(2,3),
          T(3,0), T(3,1), T(3,2), T(3,3);
  writeLineToFile(line, out_file, line_number);
}

float compareResultFiles(string file1, string file2)
{
  ifstream file1_stream, file2_stream;

  file1_stream.open( file1.c_str(), ios::binary );
  file2_stream.open( file2.c_str(), ios::binary );

  if (!file1_stream)
  {
    cout << "Couldn't open file " << file1 << endl;
    return -1;
  }
  if (!file2_stream)
  {
    cout << "Couldn't open file " << file2 << endl;
    return -1;
  }

  // Compare number of lines
  int c1, c2;
  c1 = 0;
  c2 = 0;
  string str;
  while(!file1_stream.eof())
  {
    getline(file1_stream, str);
    c1++;
  }
  while(!file2_stream.eof())
  {
    getline(file2_stream, str);
    c2++;
  }

  file1_stream.clear();
  file1_stream.seekg(0,ios::beg);

  file2_stream.clear();
  file2_stream.seekg(0,ios::beg);

  if(c1 != c2)
  {
    cout << "Number of lines in files does not match." << "\n";
    cout << file1 << " has " << c1 << " lines, while "<< file2 << " has " << c2 << " lines." << "\n";
    return -1;
  }

  // Compare files line by line
  char string1[256], string2[256];
  int j = 0, error_count = 0;
  while(!file1_stream.eof())
  {
    file1_stream.getline(string1, 256);
    file2_stream.getline(string2, 256);
    j++;
    if(strcmp(string1, string2) != 0)
    {
      cout << j << "-th strings are not equal: " << endl;
      cout << " file1: " << string1 << endl;
      cout << " file2: " << string2 << endl;
      error_count++;
    }
  }

  return (float(j - error_count) * 100.0) / float(j); 
}