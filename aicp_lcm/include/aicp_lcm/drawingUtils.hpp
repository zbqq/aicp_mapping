#ifndef DRAWING_UTILS_HPP_
#define DRAWING_UTILS_HPP_

//LCM
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/octomap_raw_t.h>

//Pronto
#include <pronto_vis/pronto_vis.hpp> // visualize point clouds

//Octomap
#include <octomap/octomap.h>
#include <octomap_utils/octomap_utils.hpp>
#include <octomap/ColorOcTree.h>

using namespace std;
using namespace Eigen;

void drawPointCloudCollections(boost::shared_ptr<lcm::LCM> &lcm, int index, Eigen::Isometry3d& pose, pcl::PointCloud<pcl::PointXYZRGB>& pcl_cloud, long long int utime, std::string pc_name_root = "Point Cloud");

void drawPointCloudNormalsCollections(boost::shared_ptr<lcm::LCM> &lcm, int index, Eigen::Isometry3d& pose, pcl::PointCloud<pcl::PointXYZRGBNormal>& pcl_cloud, long long int utime, std::string pc_name_root = "Normals");

void drawFrameCollections(boost::shared_ptr<lcm::LCM> &lcm, int index, Eigen::Isometry3d& pose, long long int utime, std::string frame_name_root = "Frame");

void HSVtoRGB( float &r, float &g, float &b, float h, float s, float v );

void publishOctreeToLCM(boost::shared_ptr<lcm::LCM> &lcm, octomap::ColorOcTree* tree, string octree_channel);
#endif
