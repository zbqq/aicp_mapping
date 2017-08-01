#ifndef FILTERING_UTILS_HPP_
#define FILTERING_UTILS_HPP_

#include <algorithm>    // std::min

//PCL
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/pca.h>
#include <pcl/visualization/pcl_visualizer.h>

void regionGrowingUniformPlaneSegmentationFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
                                                 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out);
void regionGrowingUniformPlaneSegmentationFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
                                                 pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_sampled_out,
                                                 Eigen::Isometry3d view_point,
                                                 std::vector<pcl::PointIndices>& clusters);

float overlapFilter(pcl::PointCloud<pcl::PointXYZ>& cloudA, pcl::PointCloud<pcl::PointXYZ>& cloudB,
                   Eigen::Isometry3d poseA, Eigen::Isometry3d poseB,
                   float range, float angularView,
                   pcl::PointCloud<pcl::PointXYZ>& accepted_pointsA,
                   pcl::PointCloud<pcl::PointXYZ>& accepted_pointsB);

float alignabilityFilter(pcl::PointCloud<pcl::PointXYZ>& cloudA, pcl::PointCloud<pcl::PointXYZ>& cloudB,
                         Eigen::Isometry3d poseA, Eigen::Isometry3d poseB,
                         pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudA_planes, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudB_planes,
                         pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr eigenvectors);
void computeNormalsCentroid(pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud, Eigen::Vector3f& centroid);
float overlapBoxFilter(pcl::PointCloud<pcl::PointXYZRGBNormal>& planeA, pcl::PointCloud<pcl::PointXYZRGBNormal>& planeB);

float registrationFailurePredictionFilter(Eigen::MatrixXf system_covariance);
#endif
