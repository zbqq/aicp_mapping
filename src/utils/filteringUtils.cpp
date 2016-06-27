#include <iostream>
#include "filteringUtils.hpp"

using namespace Eigen;

void planeModelSegmentationFilter(DP &dp_cloud_blob)
{
  pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
  fromDataPointsToPCL(dp_cloud_blob, pcl_cloud);

  planeModelSegmentationFilter(pcl_cloud);

  DP dp_cloud;
  fromPCLToDataPoints(dp_cloud, pcl_cloud);
  dp_cloud_blob = dp_cloud;
}

void planeModelSegmentationFilter(pcl::PointCloud<pcl::PointXYZRGB>& cloud_blob)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_blob_ptr;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>),
  cloud_p (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>),
  cloud_planes (new pcl::PointCloud<pcl::PointXYZRGB>);

  std::cerr << "Before downsampling: " << cloud_blob.width * cloud_blob.height << " data points." << std::endl;
  //std::cerr << "Before downsampling: " << pcl_cloud->width * pcl_cloud->height << " data points." << std::endl;

  // Filter: uniform distribution of points
  // Create the filtering object: downsample the dataset using a leaf size of 8cm
  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  cloud_blob_ptr = cloud_blob.makeShared();
  sor.setInputCloud(cloud_blob_ptr);
  sor.setLeafSize (0.08f, 0.08f, 0.08f);
  sor.filter (*cloud_filtered);

  std::cerr << "After downsampling: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;
  // Write the downsampled version to disk
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZRGB> ("downsampled_cloud.pcd", *cloud_filtered, false);

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.08);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;

  int i = 0, nr_points = (int) cloud_filtered->points.size ();
  // While 30% of the original cloud is still there
  while (cloud_filtered->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the inliers
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);

    /*
    Eigen::Vector4f normal;
    normal << coefficients->values[0], coefficients->values[1],
                              coefficients->values[2], coefficients->values[3];
    double area = compute2DPolygonalArea(*cloud_p, normal);*/

    //std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points. Area: " << area << std::endl;
    //std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << std::endl;

    //srand (time(NULL));
    // Cluster color
    float r = (rand() % 256);
    float g = (rand() % 256);
    float b = (rand() % 256);
    for (size_t i_point = 0; i_point < cloud_p->points.size (); i_point++)
    {
      cloud_p->points[i_point].r = r;
      cloud_p->points[i_point].g = g;
      cloud_p->points[i_point].b = b;
    }

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_f);
    cloud_filtered.swap (cloud_f);
    i++;

    // Merge the extracted planes in a single cloud
    //if (area > 0.5)
    //if((cloud_p->width * cloud_p->height) >= (cloud_filtered->width * cloud_filtered->height * 0.01))
    *cloud_planes = *cloud_planes + *cloud_p;
  }

  writer.write<pcl::PointXYZRGB> ("planes_extraction_filtered_cloud.pcd", *cloud_planes, false);

  cloud_blob = *cloud_planes;
}

void regionGrowingPlaneSegmentationFilter(DP &dp_cloud_blob)
{
  pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
  fromDataPointsToPCL(dp_cloud_blob, pcl_cloud);

  regionGrowingPlaneSegmentationFilter(pcl_cloud);

  DP dp_cloud;
  fromPCLToDataPoints(dp_cloud, pcl_cloud);
  dp_cloud_blob = dp_cloud;
}

void regionGrowingPlaneSegmentationFilter(pcl::PointCloud<pcl::PointXYZRGB>& cloud_blob)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_blob_ptr;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_planes (new pcl::PointCloud<pcl::PointXYZRGB>);

  // Filter: uniform distribution of points
  // Create the filtering object: downsample the dataset using a leaf size of 8cm
  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  cloud_blob_ptr = cloud_blob.makeShared();
  sor.setInputCloud(cloud_blob_ptr);
  sor.setLeafSize (0.08f, 0.08f, 0.08f);
  sor.filter (*cloud_filtered);

  pcl::search::Search<pcl::PointXYZRGB>::Ptr tree =
  boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);
  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (cloud_filtered);
  normal_estimator.setKSearch (30);
  normal_estimator.compute (*normals);

  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (cloud_filtered);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  pass.filter (*indices);

  pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
  reg.setMinClusterSize (50);
  reg.setMaxClusterSize (30000);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (15);
  reg.setInputCloud (cloud_filtered);
  //reg.setIndices (indices);
  reg.setInputNormals (normals);
  reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
  reg.setCurvatureThreshold (1.0);

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);

  std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
  std::cout << "First cluster has " << clusters[0].indices.size () << " points." << endl;

  srand (time(NULL));
  for (int i = 0; i < clusters.size(); i++)
  {
    pcl::PointCloud<pcl::PointXYZRGB> cloud_cluster (*cloud_filtered, clusters[i].indices);
    // Cluster color
    float r = (rand() % 256);
    float g = (rand() % 256);
    float b = (rand() % 256);
    for (size_t i_point = 0; i_point < cloud_cluster.points.size (); i_point++)
    {
      cloud_cluster.points[i_point].r = r;
      cloud_cluster.points[i_point].g = g;
      cloud_cluster.points[i_point].b = b;
    }

    *cloud_planes = *cloud_planes + cloud_cluster;
  }

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZRGB> ("region_growing_clusters.pcd", *colored_cloud, false);
  writer.write<pcl::PointXYZRGB> ("region_growing_filtered_cloud.pcd", *cloud_planes, false);

  cloud_blob = *cloud_planes;
}


double compute2DPolygonalArea (pcl::PointCloud<pcl::PointXYZRGB> cloud, Eigen::Vector4f normal)
{
  int k0, k1, k2;

  // Find axis with largest normal component and project onto perpendicular plane
  k0 = (fabs (normal[0]) > fabs (normal[1])) ? 0  : 1;
  k0 = (fabs (normal[k0]) > fabs (normal[2])) ? k0 : 2;
  k1 = (k0 + 1) % 3;
  k2 = (k0 + 2) % 3;

  // cos(theta), where theta is the angle between the polygon and the projected plane
  double ct = fabs (normal[k0]);

  double area = 0;
  float p_i[3], p_j[3];

  for (unsigned int i = 0; i < cloud.points.size (); i++)
  {
    p_i[0] = cloud.points[i].x; p_i[1] = cloud.points[i].y; p_i[2] = cloud.points[i].z;
    int j = (i + 1) % cloud.points.size ();
    p_j[0] = cloud.points[j].x; p_j[1] = cloud.points[j].y; p_j[2] = cloud.points[j].z;

    area += p_i[k1] * p_j[k2] - p_i[k2] * p_j[k1];
  }
  area = fabs (area) / (2 * ct);

  return (area);
}