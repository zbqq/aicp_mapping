// aicp-registration

// Test 3D point clouds alignment
//
// Options: - takes 2 clouds from user and gives
//            transformation between them (-a reference, -b reading)

#include <sstream>  // stringstream

// Project lib
#include "aicpRegistration/registration.hpp"
#include "aicpRegistration/common.hpp"

#include "commonUtils/cloudIO.h"
#include "commonUtils/common.hpp"

// yaml
#include "yaml-cpp/yaml.h" // read the yaml config

// pcl
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

// args
#include <ConciseArgs>

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
    cout << "[Registration] Loading point cloud A..." << endl;
    if (pcl::io::loadPCDFile (cl_cfg.pointCloudA, point_cloud_A) == -1) {
      cerr << "Was not able to open file \""<<cl_cfg.pointCloudA<<"\"." << endl;
      return EXIT_SUCCESS;
    }
    cout << "[Registration] Point cloud A loaded." << endl;
  }
  else {
    cout << "Please specify point cloud file." << endl;
    return EXIT_SUCCESS;
  }

  pcl::PointCloud<pcl::PointXYZ>& point_cloud_B = *point_cloud_B_ptr;
  if (cl_cfg.pointCloudB.compare("") != 0) {
    cout << "[Registration] Loading point cloud B..." << endl;
    if (pcl::io::loadPCDFile (cl_cfg.pointCloudB, point_cloud_B) == -1) {
      cerr << "Was not able to open file \""<<cl_cfg.pointCloudB<<"\"." << endl;
      return EXIT_SUCCESS;
    }
    cout << "[Registration] Point cloud B loaded." << endl;
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
    else if(key.compare("loadPosesFromFile") == 0) {
      params.loadPosesFromFile = it->second.as<string>();
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
      else if(key.compare("initialTransform") == 0) {
        params.pointmatcher.initialTransform = it->second.as<string>();
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

  cout << "Registration Type: "                 << params.type                          << endl;
  cout << "Load Poses from File: "              << params.loadPosesFromFile             << endl;
  cout << "Save Corrected Pose: "               << params.saveCorrectedPose             << endl;
  cout << "Save Initialized Reading Cloud: "    << params.saveInitializedReadingCloud   << endl;
  cout << "Save Registered Reading Cloud: "     << params.saveRegisteredReadingCloud    << endl;

  if(params.type.compare("Pointmatcher") == 0) {
    cout << "Config File Name: "                << params.pointmatcher.configFileName   << endl;
    cout << "Initial Transform: "               << params.pointmatcher.initialTransform << endl;
    cout << "Print Registration Statistics: "   << params.pointmatcher.printOutputStatistics << endl;
  }
  else if(params.type.compare("GICP") == 0) {
  }
  cout << "============================" << endl;

  /*===================================
  =          Load Input Pose          =
  ===================================*/

  Eigen::Matrix4f ground_truth_sensor_pose = Eigen::Matrix4f::Identity(4,4);
  if (!params.loadPosesFromFile.empty())
  {
     std::stringstream ss(extract_ints(cl_cfg.pointCloudB));
     std::istringstream iss(ss.str());
     int point_cloud_B_number;
     iss >> point_cloud_B_number;
     string line_from_file = readLineFromFile(params.loadPosesFromFile, point_cloud_B_number);
     ground_truth_sensor_pose = parseTransformationDeg(line_from_file);
     cout << "============================" << endl
          << "Ground Truth Sensor Pose:" << endl
          << "============================" << endl
          << ground_truth_sensor_pose << endl;
  }

  /*===================================
  =          Register Clouds          =
  ===================================*/

  Eigen::Matrix4f T = Eigen::Matrix4f::Zero(4,4);

  std::unique_ptr<AbstractRegistrator> registration = create_registrator(params);
  registration->registerClouds(point_cloud_A, point_cloud_B, T);

  cout << "============================" << endl
       << "Computed 3D Transform:" << endl
       << "============================" << endl
       << T << endl;

  /*===================================
  =            Save Clouds            =
  ===================================*/
  pcl::PCDWriter writer;
  if (params.saveInitializedReadingCloud) {
    pcl::PointCloud<pcl::PointXYZ> initialized_reading_cloud;
    registration->getInitializedReading(initialized_reading_cloud);
    stringstream ss;
    ss << "initialized_reading_cloud.pcd";
    writer.write<pcl::PointXYZ> (ss.str (), initialized_reading_cloud, false);
  }
  if (params.saveRegisteredReadingCloud) {
    pcl::PointCloud<pcl::PointXYZ> registered_reading_cloud;
    registration->getOutputReading(registered_reading_cloud);
    stringstream ss;
    ss << "registered_reading_cloud.pcd";
    writer.write<pcl::PointXYZ> (ss.str (), registered_reading_cloud, false);
  }

  if (params.saveCorrectedPose) {
    Eigen::Matrix4f corrected_sensor_pose;
    corrected_sensor_pose = T * ground_truth_sensor_pose;
    cout << "============================" << endl
         << "Corrected Sensor Pose:" << endl
         << "============================" << endl
         << corrected_sensor_pose << endl;
  }
  
  return 0;
}
