#include "filteringUtils.hpp"

void planeModelSegmentationFilter(pcl::PointCloud<pcl::PointXYZRGB>& cloud_blob)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_blob_ptr;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>),
  cloud_p (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>),
  cloud_planes (new pcl::PointCloud<pcl::PointXYZRGB>);

  std::cerr << "Before downsampling: " << cloud_blob.width * cloud_blob.height << " data points." << std::endl;
  //std::cerr << "Before downsampling: " << pcl_cloud->width * pcl_cloud->height << " data points." << std::endl;

  // Filter: downsampling the cloud (reduce the number of points) using a voxelized grid approach.
  // The VoxelGrid class creates a 3D voxel grid over the point cloud. In each voxel, all the points
  // will be approximated with their centroid.
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

  // Filter: RANSAC approach for  model fitting. We specify a distance threshold,
  // which determines how close a point must be to the model in order to be considered an inlier.
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

void regionGrowingPlaneSegmentationFilter(pcl::PointCloud<pcl::PointXYZRGB>& cloud_blob)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_blob_ptr;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_planes (new pcl::PointCloud<pcl::PointXYZRGB>);

  // Filter: downsampling the cloud (reduce the number of points) using a voxelized grid approach.
  // The VoxelGrid class creates a 3D voxel grid over the point cloud. In each voxel, all the points
  // will be approximated with their centroid.
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

  // Filter: It sorts the points by their curvature value and the region begins its growth
  // from the point that has the minimum curvature value. The picked point is added to the set called
  // seeds. For every seed point the algorithm finds neighbouring points and tests them
  // for the angle between their normal and normal of the current seed point.
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
  //writer.write<pcl::PointXYZRGB> ("region_growing_clusters.pcd", *colored_cloud, false);
  //writer.write<pcl::PointXYZRGB> ("region_growing_filtered_cloud.pcd", *cloud_planes, false);

  cloud_blob = *cloud_planes;
}

// The overlapFilter does not reduce the number of points in the clouds. However it computes
// a parameter describing the overlap between the two clouds.
// Input: two clouds cloudA and cloudB in local reference frame,
//        the transformations poseA and poseB (robot pose at capturing time wrt local),
//        sensor range [meters] and angular field of view [degrees].
// Output: overlapParam.
float overlapFilter(pcl::PointCloud<pcl::PointXYZ>& cloudA, pcl::PointCloud<pcl::PointXYZ>& cloudB,
                   Eigen::Isometry3d poseA, Eigen::Isometry3d poseB,
                   float range, float angularView,
                   pcl::PointCloud<pcl::PointXYZ>& accepted_pointsA,
                   pcl::PointCloud<pcl::PointXYZ>& accepted_pointsB)
{
  float thresh = (180.0-((360.0-angularView)/2));
  Eigen::Isometry3d poseBinverse;
  poseBinverse = poseB.inverse();
  // Filter 1: first cloud wrt second pose
  //pcl::PointCloud<pcl::PointXYZ> accepted_pointsA;
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
    //cout << "r: " << r << ", theta: " << theta << ", phi: " << phi << endl;
    if ((theta < thresh && theta > -thresh) && r < range)
    {
      // Store treimmed cloud, transformed back to local reference frame
      pcl::PointXYZ pointA_local;
      Eigen::Affine3d poseB_affine;
      poseB_affine.translation() = poseB.translation();
      poseB_affine.linear() = poseB.rotation();
      pointA_local = pcl::transformPoint(pointA_B, poseB_affine);
      accepted_pointsA.push_back(pointA_local);
    }
  }

  Eigen::Isometry3d poseAinverse;
  poseAinverse = poseA.inverse();
  // Filter 2: second cloud wrt first pose
  //pcl::PointCloud<pcl::PointXYZ> accepted_pointsB;
  for (int i=0; i < cloudB.size(); i++)
  {
    pcl::PointXYZ pointB_A;
    pointB_A.x = static_cast<float> (poseAinverse(0, 0) * cloudB.points[i].x + poseAinverse(0, 1) * cloudB.points[i].y + poseAinverse(0, 2) * cloudB.points[i].z + poseAinverse(0, 3));
    pointB_A.y = static_cast<float> (poseAinverse(1, 0) * cloudB.points[i].x + poseAinverse(1, 1) * cloudB.points[i].y + poseAinverse(1, 2) * cloudB.points[i].z + poseAinverse(1, 3));
    pointB_A.z = static_cast<float> (poseAinverse(2, 0) * cloudB.points[i].x + poseAinverse(2, 1) * cloudB.points[i].y + poseAinverse(2, 2) * cloudB.points[i].z + poseAinverse(2, 3));
    //pointB_A = poseB.inverse() * cloudB.points[i];

    float r = sqrt( pow(pointB_A.x,2.0) + pow(pointB_A.y,2.0) + pow(pointB_A.z,2.0) );
    float theta = atan2 (pointB_A.y,pointB_A.x) * 180 / M_PI; //degrees
    float phi = acos (pointB_A.z / r) * 180.0 / M_PI;

    // Filter out points with (theta > 225deg and theta < 315deg) and the far ones
    //cout << "r: " << r << ", theta: " << theta << ", phi: " << phi << endl;
    if ((theta < thresh && theta > -thresh) && r < range)
    {
      // Store trimmed cloud, transformed back to local reference frame
      pcl::PointXYZ pointB_local;
      Eigen::Affine3d poseA_affine;
      poseA_affine.translation() = poseA.translation();
      poseA_affine.linear() = poseA.rotation();
      pointB_local = pcl::transformPoint(pointB_A, poseA_affine);
      accepted_pointsB.push_back(pointB_local);
    }
  }

  //Compute overlap parameter dependent on percentage of accepted points per cloud
  float perc_accepted_A, perc_accepted_B;
  perc_accepted_A = (float)(accepted_pointsA.size()) / (float)(cloudA.size());
  perc_accepted_B = (float)(accepted_pointsB.size()) / (float)(cloudB.size());
  //cout << "Points left in cloudA: " << perc_accepted_A*100.0 << "%" << endl;
  //cout << "Points left in cloudB: " << perc_accepted_B*100.0 << "%" << endl;

  float overlap = perc_accepted_A * perc_accepted_B;
  //cout << "Overlap: " << overlap*100.0 << "%" << endl;

  pcl::PCDWriter writer;
  //writer.write<pcl::PointXYZ> ("accepted_pointsA.pcd", accepted_pointsA, false);
  //writer.write<pcl::PointXYZ> ("accepted_pointsB.pcd", accepted_pointsB, false);

  return overlap*100.0;
}

// from "Geometrically Stable Sampling for the ICP Algorithm", J. Gelfand et al., 2003
// from "On Degeneracy of Optimization-based State Estimation Problems", J. Zhang, 2016
float registrationFailurePredictionFilter(Eigen::MatrixXf system_covariance)
{
  Eigen::EigenSolver<Eigen::MatrixXf> es(system_covariance);
  float sum_lambda = es.eigenvalues()(0,0).real()+es.eigenvalues()(1,0).real()+es.eigenvalues()(2,0).real()+
                     es.eigenvalues()(3,0).real()+es.eigenvalues()(4,0).real()+es.eigenvalues()(5,0).real();
  Eigen::VectorXf system_lambdas(6);
  system_lambdas << (es.eigenvalues()(0,0).real()/sum_lambda), //roll
                    (es.eigenvalues()(1,0).real()/sum_lambda), //pitch
                    (es.eigenvalues()(2,0).real()/sum_lambda), //yaw
                    (es.eigenvalues()(3,0).real()/sum_lambda), //x
                    (es.eigenvalues()(4,0).real()/sum_lambda), //y
                    (es.eigenvalues()(5,0).real()/sum_lambda); //z
  cout << "[Filtering Utils] Prediction Eigenvectors:" << endl << es.eigenvectors().real().transpose() << endl;
  //cout << "[Filtering Utils] Prediction Eigenvalues:" << endl << es.eigenvalues() << endl;
  cout << "[Filtering Utils] Normalized Prediction Eigenvalues:" << endl << system_lambdas << endl;

  int pos_min, pos_max;
  float prediction;

  // Degeneracy
  // used in "On Degeneracy of Optimization-based State Estimation Problems", J. Zhang, 2016
  system_lambdas.tail<3>().minCoeff(&pos_min); // minimum eigenvalue between x, y, z only
  prediction = system_lambdas.tail<3>()[pos_min]*100.0;
  cout << "[Filtering Utils] Degeneracy (degenerate if ~ 0): " << prediction << endl;

  // Condition Number
  // used in "Geometrically Stable Sampling for the ICP Algorithm", J. Gelfand et al., 2003
  system_lambdas.tail<3>().maxCoeff(&pos_max); // maximum eigenvalue between x, y, z only
  prediction = system_lambdas.tail<3>()[pos_max]/system_lambdas.tail<3>()[pos_min];
  cout << "[Filtering Utils] Condition Number (degenerate if big, want 1): " << prediction << endl;

  // Inverse Condition Number
  // compared against in
  // "On Degeneracy of Optimization-based State Estimation Problems", J. Zhang, 2016
  prediction = system_lambdas.tail<3>()[pos_min]/system_lambdas.tail<3>()[pos_max];
  cout << "[Filtering Utils] Inverse Condition Number (degenerate if ~ 0, want 1): " << prediction << endl;

  return prediction;
}
