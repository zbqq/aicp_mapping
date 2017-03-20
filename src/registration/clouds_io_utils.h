#include <pcl/point_types.h>
#include <fstream>
#include <iostream>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>

int savePlanarCloudCSV (const std::string &file_name, const pcl::PCLPointCloud2 &cloud);

void savePointCloudPCLwithPose(const std::string file_name, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, Eigen::Isometry3d sensor_pose = Eigen::Isometry3d::Identity());