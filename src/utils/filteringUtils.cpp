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

  //std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
  //std::cout << "First cluster has " << clusters[0].indices.size () << " points." << endl;

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

// The overlapFilter does not reduce the number of points in the clouds. However it computes
// a parameter describing the overlap between the two clouds.
// Input: two clouds cloudA and cloudB in local reference frame,
//        the transformations poseA and poseB (robot pose at capturing time wrt local).
// Output: overlapParam.
void overlapFilter(DP& cloudA, DP& cloudB,
                   Eigen::Isometry3d poseA, Eigen::Isometry3d poseB)
{
  pcl::PointCloud<pcl::PointXYZRGB> pcl_cloudA, pcl_cloudB;
  fromDataPointsToPCL(cloudA, pcl_cloudA);
  fromDataPointsToPCL(cloudB, pcl_cloudB);

  overlapFilter(pcl_cloudA, pcl_cloudB, poseA, poseB);

  DP dp_cloudA, dp_cloudB;
  fromPCLToDataPoints(dp_cloudA, pcl_cloudA);
  fromPCLToDataPoints(dp_cloudB, pcl_cloudB);
  cloudA = dp_cloudA;
  cloudB = dp_cloudB;
}

void overlapFilter(pcl::PointCloud<pcl::PointXYZRGB>& cloudA, pcl::PointCloud<pcl::PointXYZRGB>& cloudB,
                   Eigen::Isometry3d poseA, Eigen::Isometry3d poseB)
{
  float range = 20.0;
  float ang_view = 270.0;
  Eigen::Isometry3d poseBinverse;
  poseBinverse = poseB.inverse();
  // Filter 1
  std::vector<pcl::PointXYZ> accepted_pointsA;
  for (int i=0; i < cloudA.size(); i++)
  {
    pcl::PointXYZ pointA_B;
    pointA_B.x = static_cast<float> (poseBinverse(0, 0) * cloudA.points[i].x + poseBinverse(0, 1) * cloudA.points[i].y + poseBinverse(0, 2) * cloudA.points[i].z + poseBinverse(0, 3));
    pointA_B.y = static_cast<float> (poseBinverse(1, 0) * cloudA.points[i].x + poseBinverse(1, 1) * cloudA.points[i].y + poseBinverse(1, 2) * cloudA.points[i].z + poseBinverse(1, 3));
    pointA_B.z = static_cast<float> (poseBinverse(2, 0) * cloudA.points[i].x + poseBinverse(2, 1) * cloudA.points[i].y + poseBinverse(2, 2) * cloudA.points[i].z + poseBinverse(2, 3));
    //pointA_B = poseB.inverse() * cloudA.points[i];

    float r = sqrt( pow(pointA_B.x,2.0) + pow(pointA_B.y,2.0) + pow(pointA_B.z,2.0) );
    float theta = atan2 (pointA_B.y,pointA_B.x) * 180 / M_PI; //degrees
    float phi = acos (pointA_B.z / r) * 180.0 / M_PI;

    // Filter out points with (theta > 225deg and theta < 315deg) and the far ones
    if ((theta < (180.0-((360.0-ang_view)/2)) || theta > (180.0+((360.0-ang_view)/2))) && r < range)
    {
      //cout << "r: " << r << ", theta: " << theta << ", phi: " << phi << endl;
      pcl::PointXYZ point;
      point.x = r * cos( theta * M_PI / 180.0 ) * sin( phi * M_PI / 180.0 );
      point.y = r * sin( theta * M_PI / 180.0 ) * sin( phi * M_PI / 180.0 );
      point.z = r * cos( phi * M_PI / 180.0 );
      accepted_pointsA.push_back(point);
    }
  }

  cout << "Points left in cloudA: " << accepted_pointsA.size() << " of " << cloudA.size() << endl;
  //cout << "Points left in cloudB: " << pointsBred_poseB.size() << " of " << transf_cloudB_poseB->size() << endl;
}

void transormCloud(pcl::PointCloud<pcl::PointXYZRGB>& cloud, 
                   pcl::PointCloud<pcl::PointXYZRGB>& transformed_cloud, Eigen::Isometry3d pose)
{
  Eigen::Affine3d poseaff;
  poseaff.translation() = pose.translation();
  poseaff.linear() = pose.rotation();

  // Executing the transformation
  pcl::transformPointCloud (cloud, transformed_cloud, poseaff);
}

void fieldOfViewFilter(std::vector<pcl::PointXYZ> &points_in, 
                       std::vector<pcl::PointXYZ> &points_out, int ang_view, int range)
{
  // cartesian to spherical coord
  for (int i=0; i < points_in.size(); i++)
  {
    float r = sqrt( pow(points_in.at(i).x,2.0) + pow(points_in.at(i).y,2.0) + pow(points_in.at(i).z,2.0) );
    float theta = atan2 (points_in.at(i).y,points_in.at(i).x) * 180 / M_PI; //degrees
    float phi = acos (points_in.at(i).z / r) * 180.0 / M_PI;

    // Filter out points with (theta > 225deg and theta < 315deg) and the far ones
    if ((theta < ang_view/2 || theta > ang_view/2) && r < range)
    {
      //cout << "r: " << r << ", theta: " << theta << ", phi: " << phi << endl;
      pcl::PointXYZ point;
      point.x = r * cos( theta * M_PI / 180.0 ) * sin( phi * M_PI / 180.0 );
      point.y = r * sin( theta * M_PI / 180.0 ) * sin( phi * M_PI / 180.0 );
      point.z = r * cos( phi * M_PI / 180.0 );
      points_out.push_back(point);
    }
  }
}


//%%%%%%%%%%%%
/*
void overlapFilter(pcl::PointCloud<pcl::PointXYZRGB>& cloudA, pcl::PointCloud<pcl::PointXYZRGB>& cloudB,
                   Eigen::Isometry3d poseA, Eigen::Isometry3d poseB)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transf_cloudA_poseA (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transf_cloudA_poseB (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transf_cloudB_poseA (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transf_cloudB_poseB (new pcl::PointCloud<pcl::PointXYZRGB> ());

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudA_ptr;

  // Transform both cloud in first reference frame
  transormCloud(cloudA, *transf_cloudA_poseA, poseA);
  transormCloud(cloudB, *transf_cloudB_poseA, poseA);
  // Filter 1
  std::vector<pcl::PointXYZ> pointsA_poseA;
  for (int i=0; i < transf_cloudA_poseA->size(); i++)
  {
    pcl::PointXYZ point;
    point.x = transf_cloudA_poseA->points[i].x;
    point.y = transf_cloudA_poseA->points[i].y;
    point.z = transf_cloudA_poseA->points[i].z;
    pointsA_poseA.push_back(point);
  }
  std::vector<pcl::PointXYZ> pointsB_poseA;
  for (int i=0; i < transf_cloudB_poseA->size(); i++)
  {
    pcl::PointXYZ point;
    point.x = transf_cloudB_poseA->points[i].x;
    point.y = transf_cloudB_poseA->points[i].y;
    point.z = transf_cloudB_poseA->points[i].z;
    pointsB_poseA.push_back(point);
  }
  std::vector<pcl::PointXYZ> pointsAred_poseA, pointsBred_poseA;
  fieldOfViewFilter(pointsA_poseA, pointsAred_poseA, 270, 20);
  fieldOfViewFilter(pointsB_poseA, pointsBred_poseA, 270, 20);

  // Transform both cloud in second reference frame
  transormCloud(*transf_cloudA_poseA, *transf_cloudA_poseB, poseB * poseA.inverse());
  transormCloud(*transf_cloudB_poseA, *transf_cloudB_poseB, poseB * poseA.inverse());
  // Filter 2
  std::vector<pcl::PointXYZ> pointsA_poseB;
  for (int i=0; i < transf_cloudA_poseB->size(); i++)
  {
    pcl::PointXYZ point;
    point.x = transf_cloudA_poseB->points[i].x;
    point.y = transf_cloudA_poseB->points[i].y;
    point.z = transf_cloudA_poseB->points[i].z;
    pointsA_poseB.push_back(point);
  }
  std::vector<pcl::PointXYZ> pointsB_poseB;
  for (int i=0; i < transf_cloudB_poseB->size(); i++)
  {
    pcl::PointXYZ point;
    point.x = transf_cloudB_poseB->points[i].x;
    point.y = transf_cloudB_poseB->points[i].y;
    point.z = transf_cloudB_poseB->points[i].z;
    pointsB_poseB.push_back(point);
  }
  std::vector<pcl::PointXYZ> pointsAred_poseB, pointsBred_poseB;
  fieldOfViewFilter(pointsA_poseB, pointsAred_poseB, 270, 20);
  fieldOfViewFilter(pointsB_poseB, pointsBred_poseB, 270, 20);

  cout << "Points left in cloudA: " << pointsAred_poseB.size() << " of " << transf_cloudA_poseB->size() << endl;
  cout << "Points left in cloudB: " << pointsBred_poseB.size() << " of " << transf_cloudB_poseB->size() << endl;

  
  // Visualization
  printf(  "\nPoint cloud colors :  white  = original point cloud\n"
      "                        red  = transformed point cloud\n");
  pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");

   // Define R,G,B colors for the point cloud
  cloudA_ptr = cloudA.makeShared();
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> source_cloud_color_handler (transf_cloudA_poseA, 255, 255, 255);
  // We add the point cloud to the viewer and pass the color handler
  viewer.addPointCloud (transf_cloudA_poseA, source_cloud_color_handler, "original_cloud");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> transformed_cloud_color_handler (transf_cloudA_poseB, 230, 20, 20); // Red
  viewer.addPointCloud (transf_cloudA_poseB, transformed_cloud_color_handler, "transformed_cloud");

  viewer.addCoordinateSystem (1.0, 0);
  viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
  //viewer.setPosition(800, 400); // Setting visualiser window position

  while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
    viewer.spinOnce ();
  }


  cloudA = *transf_cloudA_poseA;
  cloudB = *transf_cloudB_poseA;
}*/