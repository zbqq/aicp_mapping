// Demo program for testing AICP:
//  1. Reading two point clouds from .pcd file (with sensor origin information)
//  2. AICP registration:
//      a. overlap analysis
//      b. alignability analysis
//      c. risk of alignment failure analysis (pre-trained SVM)
//      d. registration
//  3. Writing result to file and console
//  4. Compare file with expected result

// Test data at: <HOME>/drs_testing_data/aicp-data
// Run: rosrun aicp_core aicp_test

#include <pwd.h>

// Project lib
#include "aicp_registration/app.hpp"

#include "aicp_registration/yaml_configurator.hpp"
#include "aicp_utils/common.hpp"
#include "aicp_utils/cloudIO.h"

using namespace std;
using namespace aicp;

void getPoseFromPointCloud(pcl::PointCloud<pcl::PointXYZ> &cloud, Eigen::Isometry3d& pose);
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

  int number_test_clouds = 16;

  CommandLineConfig cl_cfg;
  cl_cfg.registration_config_file.append(homedir);
  cl_cfg.registration_config_file.append("/code/aicp_base/git/aicp/aicp_core/config/icp/icp_autotuned.yaml");
  cl_cfg.aicp_config_file.append(homedir);
  cl_cfg.aicp_config_file.append("/code/aicp_base/git/aicp/aicp_core/config/aicp_test_config.yaml");
  cl_cfg.localize_against_prior_map = false; // otherwise overlap set to high default value
  cl_cfg.failure_prediction_mode = true; // compute Alignment Risk
  cl_cfg.verbose = false;

  // Expected result file
  std::stringstream expected_file;
  expected_file << homedir;
  expected_file << "/drs_testing_data/aicp-data/test_aicp_expected.txt";
  // Output file
  std::stringstream out_file;
  out_file << homedir;
  out_file << "/drs_testing_data/aicp-data/test_aicp_output.txt";
  std::ofstream ofs;
  ofs.open(out_file.str().c_str(), std::ofstream::out | std::ofstream::trunc);
  ofs.close();

  /*===================================
  =            YAML Config            =
  ===================================*/
  aicp::YAMLConfigurator yaml_conf;
  if(!yaml_conf.parse(cl_cfg.aicp_config_file)){
      cerr << "ERROR: could not parse file " << cl_cfg.aicp_config_file << endl;
      return -1;
  }
  yaml_conf.printParams();

  // Instantiate AICP app
  std::shared_ptr<aicp::App> app_test(new aicp::App(cl_cfg,
                                                    yaml_conf.getRegistrationParams(),
                                                    yaml_conf.getOverlapParams(),
                                                    yaml_conf.getClassificationParams()));

  if(yaml_conf.getRegistrationParams().loadPosesFrom != "pcd"){
      cerr << "ERROR: load poses from " << yaml_conf.getRegistrationParams().loadPosesFrom
           << " not valid. Expected load poses from PCD." << endl;
      return -1;
  }
  else
      cout << "[AICP Test] Loading poses from PCD..." << endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr reference (new pcl::PointCloud<pcl::PointXYZ>);
  Eigen::Isometry3d reference_pose;

  for (int i = 0; i < number_test_clouds; i ++)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr reading (new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Isometry3d reading_pose;
  
    std::stringstream pcl_name;
    pcl_name << homedir;
    pcl_name << "/drs_testing_data/aicp-data/cloud_";
    pcl_name << std::to_string(i);
    pcl_name << ".pcd";

    if (i == 0) // load reference cloud from file
    {
      /*===================================
      =   Load Reference Cloud and Pose   =
      ===================================*/
      if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcl_name.str().c_str(), *reference) == -1)
      {
        PCL_ERROR ("Error loading PCD file.\n");
        return (-1);
      }
      std::cout << "Loaded " << reference->width * reference->height << " data points from " << pcl_name.str().c_str() << std::endl;

      getPoseFromPointCloud(*reference, reference_pose);
    }
    else // load i-th reading cloud from file
    {
      /*===================================
      =    Load Reading Cloud and Pose    =
      ===================================*/      
      if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcl_name.str().c_str(), *reading) == -1)
      {
        PCL_ERROR ("Error loading PCD file.\n");
        return (-1);
      }      
      std::cout << "Loaded " << reading->width * reading->height << " data points from " << pcl_name.str().c_str() << std::endl;

      getPoseFromPointCloud(*reading, reading_pose);

      cout << "\n" << "-------------------------------------------------------------------------------" << "\n \n";

      /*=====================================
      =          AICP Registration          =
      =====================================*/
      // Pre-filtering
      pcl::PointCloud<pcl::PointXYZ>::Ptr ref_prefiltered (new pcl::PointCloud<pcl::PointXYZ>);
      app_test->filterCloud(reference, ref_prefiltered);

      pcl::PointCloud<pcl::PointXYZ>::Ptr read_prefiltered (new pcl::PointCloud<pcl::PointXYZ>);
      app_test->filterCloud(reading, read_prefiltered);

      if(cl_cfg.verbose)
      {
          // Save filtered clouds to file
          pcl::PCDWriter pcd_writer_;
          stringstream filtered_ref;
          filtered_ref << app_test->getDataDirectoryPath();
          filtered_ref << "/reference_prefiltered.pcd";
          pcd_writer_.write<pcl::PointXYZ> (filtered_ref.str (), *ref_prefiltered, false);
          stringstream filtered_read;
          filtered_read << app_test->getDataDirectoryPath();
          filtered_read << "/reading_prefiltered.pcd";
          pcd_writer_.write<pcl::PointXYZ> (filtered_read.str (), *read_prefiltered, false);
      }

      // Registration
      Eigen::Matrix4f correction = Eigen::Matrix4f::Identity(4,4);
      app_test->runAicpPipeline(ref_prefiltered, read_prefiltered, reference_pose, reading_pose, correction);

      pcl::PointCloud<pcl::PointXYZ>::Ptr output (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::transformPointCloud (*read_prefiltered, *output, correction);

      // Write results to file
      writeResultToFile(out_file.str().c_str(), i, read_prefiltered->size(),
                        app_test->getFOVOverlap(), app_test->getOctreeOverlap(), app_test->getAlignability(),
                        app_test->getAlignmentRisk(), correction);

      cout << "\n" << "-------------------------------------------------------------------------------" << "\n \n";
    }
  }
  // Compare with expected results from file
  float match = compareResultFiles(expected_file.str().c_str(), out_file.str().c_str());
  cout << "Results Match: " << match << "%" << endl;

  return (0);
}

void getPoseFromPointCloud(pcl::PointCloud<pcl::PointXYZ>& cloud, Eigen::Isometry3d& pose)
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
