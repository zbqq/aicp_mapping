#include <iostream>
#include "filteringUtils.hpp"

using namespace Eigen;

void planesExtractionFilter(DP &dp_cloud_blob)
{
  pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
  fromDataPointsToPCL(dp_cloud_blob, pcl_cloud);

  planesExtractionFilter(pcl_cloud);
}

void planesExtractionFilter(pcl::PointCloud<pcl::PointXYZRGB>& cloud_blob)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_blob_ptr;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>),
  cloud_p (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>),
  cloud_planes (new pcl::PointCloud<pcl::PointXYZRGB>);

  std::cerr << "PointCloud before filtering: " << cloud_blob.width * cloud_blob.height << " data points." << std::endl;
  //std::cerr << "PointCloud before filtering: " << pcl_cloud->width * pcl_cloud->height << " data points." << std::endl;

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  cloud_blob_ptr = cloud_blob.makeShared();
  sor.setInputCloud(cloud_blob_ptr);
  sor.setLeafSize (0.08f, 0.08f, 0.08f);
  sor.filter (*cloud_filtered);

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

  // Write the downsampled version to disk
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZRGB> ("table_scene_lms400_downsampled.pcd", *cloud_filtered, false);

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
    std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << std::endl;
    
    std::stringstream ss;
    ss << "table_scene_lms400_plane_" << i << ".pcd";
    writer.write<pcl::PointXYZRGB> (ss.str (), *cloud_p, false);

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
/*
  // Sparse points filter
  pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
  // build the filter
  outrem.setInputCloud(cloud_planes);
  outrem.setRadiusSearch(0.1);
  outrem.setMinNeighborsInRadius (7);
  // apply filter
  outrem.filter (*cloud_planes);
*/
  writer.write<pcl::PointXYZRGB> ("table_scene_lms400_filtering.pcd", *cloud_filtered, false);
  writer.write<pcl::PointXYZRGB> ("table_scene_lms400_planes.pcd", *cloud_planes, false);
}

/////////// TO_DO: Try Region growing segmentation!!!!!!!

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