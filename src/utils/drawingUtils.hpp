#ifndef SRC_DRAWINGUTIL_HPP_
#define SRC_DRAWINGUTIL_HPP_

#include <Eigen/Dense>
#include "bot_lcmgl_client/lcmgl.h"
#include <lcm/lcm-cpp.hpp>

#include <icp-registration/icp_utils.h>
#include <pronto_utils/pronto_vis.hpp> // visualize pt clds

void drawPointCloudCollections(boost::shared_ptr<lcm::LCM> &lcm, int index, Eigen::Isometry3d& pose, DP &dp_cloud, long long int utime);
void drawPointCloudCollections(boost::shared_ptr<lcm::LCM> &lcm, int index, Eigen::Isometry3d& pose, pcl::PointCloud<pcl::PointXYZRGB>& pcl_cloud, long long int utime);

void drawPointCloudLCMGL(bot_lcmgl_t *lcmgl, std::vector<Eigen::Vector3f> point_cloud);
void drawPointCloudLCMGL(bot_lcmgl_t *lcmgl, DP &dp_cloud);
void drawPointCloudLCMGL(bot_lcmgl_t *lcmgl, pcl::PointCloud<pcl::PointXYZRGB>& pcl_cloud);

void drawFrameCollections(boost::shared_ptr<lcm::LCM> &lcm, int index, Eigen::Isometry3d& pose, long long int utime);

void drawFrameLCMGL(bot_lcmgl_t *lcmgl);
void drawFrameLCMGL(bot_lcmgl_t *lcmgl, Eigen::Isometry3d transform);
void drawFrameLCMGL(bot_lcmgl_t *lcmgl, Eigen::Vector3d origin,
	           Eigen::Vector3d x_tip, Eigen::Vector3d y_tip, Eigen::Vector3d z_tip);
void draw3dLine(bot_lcmgl_t *lcmgl, Eigen::Vector3d start, Eigen::Vector3d end);
void draw3dLine(bot_lcmgl_t *lcmgl, double start_x, double start_y, double start_z, double end_x, double end_y, double end_z);
void HSVtoRGB( float &r, float &g, float &b, float h, float s, float v );

#endif
