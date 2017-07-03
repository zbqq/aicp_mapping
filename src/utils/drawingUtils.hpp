#ifndef DRAWING_UTILS_HPP_
#define DRAWING_UTILS_HPP_

#include "bot_lcmgl_client/lcmgl.h"

//LCM
#include <lcm/lcm-cpp.hpp>

//Project lib
#include <commonUtils/cloudIO.h>

//Pronto
#include <pronto_utils/pronto_vis.hpp> // visualize point clouds

using namespace std;
using namespace Eigen;

void drawPointCloudCollections(boost::shared_ptr<lcm::LCM> &lcm, int index, Eigen::Isometry3d& pose, DP &dp_cloud, long long int utime, std::string pc_name_root = "Point Cloud");
void drawPointCloudCollections(boost::shared_ptr<lcm::LCM> &lcm, int index, Eigen::Isometry3d& pose, pcl::PointCloud<pcl::PointXYZRGB>& pcl_cloud, long long int utime, std::string pc_name_root = "Point Cloud");

void drawPointCloudLCMGL(bot_lcmgl_t *lcmgl, std::vector<Eigen::Vector3f> point_cloud);
void drawPointCloudLCMGL(bot_lcmgl_t *lcmgl, DP &dp_cloud);
void drawPointCloudLCMGL(bot_lcmgl_t *lcmgl, pcl::PointCloud<pcl::PointXYZRGB>& pcl_cloud);

void drawFrameCollections(boost::shared_ptr<lcm::LCM> &lcm, int index, Eigen::Isometry3d& pose, long long int utime, std::string frame_name_root = "Frame");

void drawFrameLCMGL(bot_lcmgl_t *lcmgl);
void drawFrameLCMGL(bot_lcmgl_t *lcmgl, Eigen::Isometry3d transform);
void drawFrameLCMGL(bot_lcmgl_t *lcmgl, Eigen::Vector3d origin,
	           Eigen::Vector3d x_tip, Eigen::Vector3d y_tip, Eigen::Vector3d z_tip);
void draw3dLine(bot_lcmgl_t *lcmgl, Eigen::Vector3d start, Eigen::Vector3d end);
void draw3dLine(bot_lcmgl_t *lcmgl, double start_x, double start_y, double start_z, double end_x, double end_y, double end_z);
void HSVtoRGB( float &r, float &g, float &b, float h, float s, float v );

#endif
