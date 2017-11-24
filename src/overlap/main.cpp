// aicp-overlap

// Computes overlap between a pair of 3D point clouds
// based on octree structures
//
// Options: - takes 2 clouds from user and computes
//            overlap between them (-a reference, -b reading)


#include <sstream>      // std::stringstream

// args
#include <ConciseArgs>

// Project lib
#include "aicpOverlap/overlap.hpp"
#include "aicpOverlap/common.hpp"

#include "aicpCommonUtils/fileIO.h"
#include "aicpCommonUtils/common.hpp"
#include "aicpDrawingUtils/drawingUtils.hpp"

// pcl
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

// yaml
#include "yaml-cpp/yaml.h" // read the yaml config

// lcm
#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/octomap_utils.h>

using namespace std;
using namespace aicp;

struct AppConfig
{
  string configFile;
  string pointCloudA;
  string pointCloudB;
};

class App{
  public:
    App(AppConfig app_cfg, boost::shared_ptr<lcm::LCM> &lcm_);
    
    ~App(){
    }  

    AppConfig app_cfg_;
    OverlapParams params_;
    
    boost::shared_ptr<lcm::LCM> lcm_;
    std::unique_ptr<AbstractOverlapper> overlapper_;

    float octree_overlap_ = -1.0;

  private:
    string yamlConfig_;
    YAML::Node yn_;

    Eigen::Matrix4f ground_truth_reference_pose_;
    Eigen::Matrix4f ground_truth_reading_pose_;

};

App::App(AppConfig app_cfg_, boost::shared_ptr< lcm::LCM >& lcm_) :
                                   app_cfg_(app_cfg_), lcm_(lcm_){
  /*===================================
  =        Load Input Clouds          =
  ===================================*/
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_A_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_B_ptr (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PointCloud<pcl::PointXYZ>& point_cloud_A = *point_cloud_A_ptr;
  if (app_cfg_.pointCloudA.compare("") != 0) {
    cout << "[Main] Loading point cloud A..." << endl;
    if (pcl::io::loadPCDFile (app_cfg_.pointCloudA, point_cloud_A) == -1) {
      cerr << "Was not able to open file \""<<app_cfg_.pointCloudA<<"\"." << endl;
    }
    cout << "[Main] Point cloud A loaded." << endl;
  }
  else {
    cout << "Please specify point cloud file." << endl;
  }

  pcl::PointCloud<pcl::PointXYZ>& point_cloud_B = *point_cloud_B_ptr;
  if (app_cfg_.pointCloudB.compare("") != 0) {
    cout << "[Main] Loading point cloud B..." << endl;
    if (pcl::io::loadPCDFile (app_cfg_.pointCloudB, point_cloud_B) == -1) {
      cerr << "Was not able to open file \""<<app_cfg_.pointCloudB<<"\"." << endl;
    }
    cout << "[Main] Point cloud B loaded." << endl;
  }
  else {
    cout << "Please specify point cloud file." << endl;
  }

  /*===================================
  =            YAML Config            =
  ===================================*/
  yamlConfig_ = app_cfg_.configFile;
  yn_ = YAML::LoadFile(yamlConfig_);

  YAML::Node overlapNode = yn_["AICP"]["Overlap"];
  for(YAML::const_iterator it=overlapNode.begin();it != overlapNode.end();++it) {

    const string key = it->first.as<string>();

    if(key.compare("type") == 0) {
      params_.type = it->second.as<string>();
    }
    else if(key.compare("loadPosesFromFile") == 0) {
      params_.loadPosesFromFile = it->second.as<string>();
    }
  }
  if(params_.type.compare("OctreeBased") == 0) {

    YAML::Node octreeBasedNode = overlapNode["OctreeBased"];

    for(YAML::const_iterator it=octreeBasedNode.begin();it != octreeBasedNode.end();++it) {
      const string key = it->first.as<string>();

      if(key.compare("octomapResolution") == 0) {
        params_.octree_based.octomapResolution = it->second.as<float>();
      }
    }
  }

  cout << "============================" << endl
       << "Parsed YAML Config" << endl
       << "============================" << endl;

  cout << "[Main] Overlap Type: "                   << params_.type                             << endl;
  cout << "[Main] Load Poses from File: "           << params_.loadPosesFromFile                << endl;

  if(params_.type.compare("OctreeBased") == 0) {
    cout << "[OctreeBased] Octomap Resolution: "    << params_.octree_based.octomapResolution   << endl;
  }
  cout << "============================" << endl;

  /*===================================
  =          Load Input Poses         =
  ===================================*/

  ground_truth_reference_pose_ = Eigen::Matrix4f::Identity(4,4);
  ground_truth_reading_pose_ = Eigen::Matrix4f::Identity(4,4);
  int point_cloud_A_number = -1;
  int point_cloud_B_number = -1;
  if (!params_.loadPosesFromFile.empty())
  {
     std::stringstream ssA(extract_ints(app_cfg_.pointCloudA));
     std::istringstream issA(ssA.str());
     issA >> point_cloud_A_number;
     string lineA_from_file = readLineFromFile(params_.loadPosesFromFile, point_cloud_A_number);
     ground_truth_reference_pose_ = parseTransformationQuaternions(lineA_from_file);

     std::stringstream ssB(extract_ints(app_cfg_.pointCloudB));
     std::istringstream issB(ssB.str());
     issB >> point_cloud_B_number;
     string lineB_from_file = readLineFromFile(params_.loadPosesFromFile, point_cloud_B_number);
     ground_truth_reading_pose_ = parseTransformationQuaternions(lineB_from_file);
     cout << "============================" << endl
          << "Ground Truth Reading Pose:" << endl
          << "============================" << endl
          << ground_truth_reading_pose_ << endl;
  }
  else
    cerr << "[Main] Input poses cannot be loaded. Overlap cannot be computed." << endl;

  /*===================================
  =          Octree Overlap           =
  ===================================*/
  overlapper_ = create_overlapper(params_);

  ColorOcTree* ref_tree;
  ColorOcTree* read_tree = new ColorOcTree(params_.octree_based.octomapResolution);

  // Create octree from reference cloud (wrt robot point of view),
  // add the reading cloud and compute overlap
  Eigen::Isometry3d ground_truth_reference_pose_iso = fromMatrix4fToIsometry3d(ground_truth_reference_pose_);
  Eigen::Isometry3d ground_truth_reading_pose_iso = fromMatrix4fToIsometry3d(ground_truth_reading_pose_);
  ref_tree = overlapper_->computeOverlap(point_cloud_A, point_cloud_B,
                                         ground_truth_reference_pose_iso, ground_truth_reading_pose_iso,
                                         read_tree);
  octree_overlap_ = overlapper_->getOverlap();

  cout << "[Main] Octrees Overlap: " << octree_overlap_ << endl;

  publishOctreeToLCM(lcm_, ref_tree, "OCTOMAP_REF");
  publishOctreeToLCM(lcm_, read_tree, "OCTOMAP");
}

int main(int argc, char **argv)
{
  //Initialization from user
  AppConfig app_cfg;
  app_cfg.configFile.append(CONFIG_LOC);
  app_cfg.configFile.append(PATH_SEPARATOR);
  app_cfg.configFile.append("aicp_config.yaml");
  app_cfg.pointCloudA = "";
  app_cfg.pointCloudB = "";

  ConciseArgs parser(argc, argv, "");
  parser.add(app_cfg.pointCloudA, "a", "point_cloud_reference", "First point cloud.");
  parser.add(app_cfg.pointCloudB, "b", "point_cloud_reading", "Second point cloud.");
  parser.parse();

  //Set up lcm channel for visualization
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"[Main] LCM is not good for visualization." <<std::endl;
  }

  App* app = new App(app_cfg, lcm);
  
  return 0;
}
