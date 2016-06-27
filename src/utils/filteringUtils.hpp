#ifndef SRC_FILTERINGUTIL_HPP_
#define SRC_FILTERINGUTIL_HPP_

#include <vector>
#include <Eigen/Dense>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>

#include <icp-registration/icp_utils.h>

void planeModelSegmentationFilter(DP &dp_cloud_blob);
void planeModelSegmentationFilter(pcl::PointCloud<pcl::PointXYZRGB>& cloud_blob);

void regionGrowingPlaneSegmentationFilter(DP &dp_cloud_blob);
void regionGrowingPlaneSegmentationFilter(pcl::PointCloud<pcl::PointXYZRGB>& cloud_blob);

double compute2DPolygonalArea (pcl::PointCloud<pcl::PointXYZRGB> cloud, Eigen::Vector4f normal);

#endif
