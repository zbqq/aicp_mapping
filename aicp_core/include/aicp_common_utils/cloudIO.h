#ifndef CLOUD_IO_HPP_
#define CLOUD_IO_HPP_

//PCL
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>

//libpointmatcher
#include "pointmatcher/PointMatcher.h"

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;
typedef PM::Parameters Parameters;

int savePlanarCloudCSV (const std::string &file_name, const pcl::PCLPointCloud2 &cloud);

void savePointCloudPCLwithPose(const std::string file_name, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, Eigen::Isometry3d sensor_pose = Eigen::Isometry3d::Identity());


void fromDataPointsToPCL(DP &cloud_in, pcl::PointCloud<pcl::PointXYZ> &cloud_out);
void fromPCLToDataPoints(DP &cloud_out, pcl::PointCloud<pcl::PointXYZ> &cloud_in);

void fromDataPointsToPCL(DP &cloud_in, pcl::PointCloud<pcl::PointXYZRGB> &cloud_out);
void fromPCLToDataPoints(DP &cloud_out, pcl::PointCloud<pcl::PointXYZRGB> &cloud_in);

void fromDataPointsToPCL(DP &cloud_in, pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_out);
void fromPCLToDataPoints(DP &cloud_out, pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_in);

PM::TransformationParameters parseTransformationDeg(std::string& transform,
                        const int cloudDimension);
PM::TransformationParameters parseTransformation(std::string& transform,
                        const int cloudDimension);

#endif
