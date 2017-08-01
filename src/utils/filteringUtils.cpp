#include "filteringUtils.hpp"

// Returns filtered cloud: uniform sampling and planes segmentation.
// This filter reduces the input's size.
void regionGrowingUniformPlaneSegmentationFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
                                                 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sampled (new pcl::PointCloud<pcl::PointXYZ>);
  // Down-sampling to get uniform points distribution.
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud_in);
  sor.setLeafSize (0.08f, 0.08f, 0.08f);
  sor.filter(*cloud_sampled);

  // Normals extraction.
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::Search<pcl::PointXYZ>::Ptr tree =
  boost::shared_ptr<pcl::search::Search<pcl::PointXYZ>> (new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod(tree);
  normal_estimator.setInputCloud(cloud_sampled);
  normal_estimator.setKSearch(30);
  normal_estimator.compute(*normals);

  // Region Growing Planes Extraction (segmentation)
  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
  reg.setMinClusterSize(50);
  reg.setMaxClusterSize(30000);
  reg.setSearchMethod(tree);
  reg.setNumberOfNeighbours(15);
  reg.setInputCloud(cloud_sampled);
  reg.setInputNormals(normals);
  reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
  reg.setCurvatureThreshold(1.0);
  std::vector <pcl::PointIndices> clusters;
  reg.extract(clusters);

  // Populate cloud with clusters
  srand (time(NULL));
  for (int i = 0; i < clusters.size(); i++)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud_cluster (*cloud_sampled, clusters[i].indices);
    *cloud_out = *cloud_out + cloud_cluster;
  }
}

// The output cloud is the input after uniform sampling
// - colors indicate the clusters after planes segmentation
// - clusters contains indices to the points in each cluster
// With this filter the input cloud remains invariate.
void regionGrowingUniformPlaneSegmentationFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
                                                 pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_sampled_out,
                                                 Eigen::Isometry3d view_point,
                                                 std::vector<pcl::PointIndices>& clusters)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sampled (new pcl::PointCloud<pcl::PointXYZ>);
  // Down-sampling to get uniform points distribution.
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud_in);
  sor.setLeafSize (0.08f, 0.08f, 0.08f);
  sor.filter(*cloud_sampled);
  pcl::copyPointCloud(*cloud_sampled, *cloud_sampled_out);

  // Normals extraction.
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::Search<pcl::PointXYZRGBNormal>::Ptr tree =
  boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGBNormal>> (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
  pcl::NormalEstimation<pcl::PointXYZRGBNormal, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod(tree);
  normal_estimator.setInputCloud(cloud_sampled_out);
  normal_estimator.setKSearch(30);
  normal_estimator.setViewPoint(view_point.translation().x(), view_point.translation().y(), view_point.translation().z());
  normal_estimator.compute(*normals);
  pcl::copyPointCloud(*normals, *cloud_sampled_out);

  // Region Growing Planes Extraction (segmentation)
  pcl::RegionGrowing<pcl::PointXYZRGBNormal, pcl::Normal> reg;
  reg.setMinClusterSize(50);
  reg.setMaxClusterSize(30000);
  reg.setSearchMethod(tree);
  reg.setNumberOfNeighbours(15);
  reg.setInputCloud(cloud_sampled_out);
  reg.setInputNormals(normals);
  reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
  reg.setCurvatureThreshold(1.0);
  reg.extract(clusters);

  // Color output cloud
  srand (time(NULL));
  for (int i = 0; i < clusters.size(); i++)
  {
    // Color segment
    uint8_t r = (rand() % 256);
    uint8_t g = (rand() % 256);
    uint8_t b = (rand() % 256);
    int32_t rgb = (r << 16) | (g << 8) | b;
    for (size_t i_point = 0; i_point < clusters[i].indices.size(); i_point++)
    {
      int idx = clusters[i].indices[i_point];
      cloud_sampled_out->points[idx].rgb = rgb;
    }
  }
}

// The overlapFilter does not reduce the number of points in the clouds. However it computes
// a parameter describing the overlap between the two clouds.
// Input: two clouds cloudA and cloudB in global reference frame,
//        the transformations poseA and poseB (robot pose at capturing time wrt global),
//        sensor range [meters] and angular field of view [degrees].
// Output: overlapParam and 2 new clouds with points belonging to overlap region.
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

// Our implementation of registrationFailurePredictionFilter
float alignabilityFilter(pcl::PointCloud<pcl::PointXYZ>& cloudA, pcl::PointCloud<pcl::PointXYZ>& cloudB,
                         Eigen::Isometry3d poseA, Eigen::Isometry3d poseB,
                         pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudA_planes, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudB_planes,
                         pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr eigenvectors)
{
  // Expected: cloudA and cloudB are points belonging to the region of overlap
  float alignability = -1.0;

  // 1. Pre-process clouds: down-sampling and planes extraction.
  // Processing cloud A...
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA_ptr;
  cloudA_ptr = cloudA.makeShared();
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudA_sampled (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  std::vector <pcl::PointIndices> clustersA;
  regionGrowingUniformPlaneSegmentationFilter(cloudA_ptr, cloudA_sampled, poseA, clustersA);
  // Processing cloud B...
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudB_ptr;
  cloudB_ptr = cloudB.makeShared();
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudB_sampled (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  std::vector <pcl::PointIndices> clustersB;
  regionGrowingUniformPlaneSegmentationFilter(cloudB_ptr, cloudB_sampled, poseB, clustersB);

  // 2. Planes matching: keep matching planes between clouds.
  // Vector with matching indeces and corresponding overlap value
  // It will contain: -1 if no correspondence exist
  //                  idx (clusterA index) otherwise
  std::vector<int> matching_indeces (clustersB.size());
  std::vector<float> matching_overlap (clustersB.size());
  std::vector<float> matching_distance (clustersB.size());
  for (int i = 0; i < clustersB.size(); i++)
  {
    matching_indeces[i] = -1;
    matching_overlap[i] = -1;
    matching_distance[i] = -1;
  }

  for (int i = 0; i < clustersA.size(); i++)
  {
    pcl::PointCloud<pcl::PointXYZRGBNormal> cloudA_cluster (*cloudA_sampled, clustersA[i].indices);

    Eigen::Vector3f clusterA_centroid;
    computeNormalsCentroid(cloudA_cluster, clusterA_centroid);

    float max_overlap = 0, current_distance = -1;
    int matching_cluster_idx = -1;
    for (int j = 0; j < clustersB.size(); j++)
    {
      pcl::PointCloud<pcl::PointXYZRGBNormal> cloudB_cluster (*cloudB_sampled, clustersB[j].indices);

      Eigen::Vector3f clusterB_centroid;
      computeNormalsCentroid(cloudB_cluster, clusterB_centroid);

      float dot = clusterA_centroid.transpose() * clusterB_centroid;
      float dist = acos( dot/(clusterA_centroid.norm()*clusterB_centroid.norm()) ) * 180.0/M_PI; // degrees

      // Planes Overlap Filter
      float current_overlap = overlapBoxFilter(cloudA_cluster, cloudB_cluster);
      if (current_overlap > max_overlap)
      {
        matching_cluster_idx = j;
        max_overlap = current_overlap;
        current_distance = dist;
      }
    }

    if (max_overlap > 40) //&& current_distance < 90) // Threshold for minimum matching overlap
                                                   // and maximum angular distance between centroid normals (deg)
    {
      if (matching_indeces.at(matching_cluster_idx) == -1)
      {
        matching_indeces[matching_cluster_idx] = i;
        matching_overlap[matching_cluster_idx] = max_overlap;
        matching_distance[matching_cluster_idx] = current_distance;
      }
      else
      {
        if (max_overlap > matching_overlap[matching_cluster_idx])
        {
          matching_indeces[matching_cluster_idx] = i;
          matching_overlap[matching_cluster_idx] = max_overlap;
          matching_distance[matching_cluster_idx] = current_distance;
        }
      }
    }
  }

  // PCA on unit sphere
  // Data Structure
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudA_normals (new pcl::PointCloud<pcl::PointXYZRGB>);

  srand (time(NULL));
  for (int i = 0; i < matching_indeces.size(); i++)
  {
    if (matching_indeces.at(i) != -1)
    {
      pcl::PointCloud<pcl::PointXYZRGBNormal> match_clusterA (*cloudA_sampled, clustersA[matching_indeces.at(i)].indices);
      pcl::PointCloud<pcl::PointXYZRGBNormal> match_clusterB (*cloudB_sampled, clustersB[i].indices);

      // Assign random color to cluster A
      uint8_t r = (rand() % 256);
      uint8_t g = (rand() % 256);
      uint8_t b = (rand() % 256);
      int32_t rgb = (r << 16) | (g << 8) | b;
      for (size_t i_point = 0; i_point < match_clusterA.points.size (); i_point++)
      {
        match_clusterA.points[i_point].rgb = rgb;
      }
      // Assign same color to matched cluster B
      for (size_t i_point = 0; i_point < match_clusterB.points.size (); i_point++)
      {
        match_clusterB.points[i_point].rgb = rgb;
      }
      *cloudA_planes = *cloudA_planes + match_clusterA;
      *cloudB_planes = *cloudB_planes + match_clusterB;
      // Store in data structures (mirrored normals as well to be able to identify constraints
      // on unit sphere using PCA)
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusterA_normals (new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusterA_normals_mirrored (new pcl::PointCloud<pcl::PointXYZRGB>);
      clusterA_normals->width = match_clusterA.size();
      clusterA_normals_mirrored->width = match_clusterA.size();
      clusterA_normals->height = 1;
      clusterA_normals_mirrored->height = 1;
      clusterA_normals->is_dense = false;
      clusterA_normals_mirrored->is_dense = false;
      clusterA_normals->points.resize(clusterA_normals->width * clusterA_normals->height);
      clusterA_normals_mirrored->points.resize(clusterA_normals_mirrored->width * clusterA_normals_mirrored->height);
      for (int i_point = 0; i_point < clusterA_normals->size(); i_point++) {
        clusterA_normals->points[i_point].x = match_clusterA.points[i_point].normal_x;
        clusterA_normals->points[i_point].y = match_clusterA.points[i_point].normal_y;
        clusterA_normals->points[i_point].z = match_clusterA.points[i_point].normal_z;
        clusterA_normals->points[i_point].rgb = match_clusterA.points[i_point].rgb;
        clusterA_normals_mirrored->points[i_point].x = - match_clusterA.points[i_point].normal_x;
        clusterA_normals_mirrored->points[i_point].y = - match_clusterA.points[i_point].normal_y;
        clusterA_normals_mirrored->points[i_point].z = - match_clusterA.points[i_point].normal_z;
        clusterA_normals_mirrored->points[i_point].rgb = match_clusterA.points[i_point].rgb;
      }

      *cloudA_normals = *cloudA_normals + *clusterA_normals;
      *cloudA_normals = *cloudA_normals + *clusterA_normals_mirrored;
    }
  }

  if(cloudA_normals->empty())
    return 0;

  // Debug
  std::cout << "matching_indeces:";
  for (unsigned i=0; i<matching_indeces.size(); i++)
    std::cout << '\t' << matching_indeces[i];
  std::cout << '\n';
  std::cout << "matching_overlap:";
  for (unsigned i=0; i<matching_overlap.size(); i++)
    std::cout << '\t' << matching_overlap[i];
  std::cout << '\n';
  std::cout << "matching_distance:";
  for (unsigned i=0; i<matching_distance.size(); i++)
    std::cout << '\t' << matching_distance[i];
  std::cout << '\n';
  std::cout << "size: " << matching_distance.size() << endl;

  // 3. Constraints Analysis on Unit Sphere:
  // compute continuous value describing "alignability".
  // Principal Component Analysis ===============================
  pcl::PCA<pcl::PointXYZRGB> cpca = new pcl::PCA<pcl::PointXYZRGB>;
  cpca.setInputCloud(cloudA_normals);
  Eigen::Vector3f pca_values;
  Eigen::Matrix3f pca_vector;
  pca_values = cpca.getEigenValues();
  // Normalize eigenvalues
  float lambda0, lambda1, lambda2; // lambda0 > lambda1 > lambda2
  lambda0 = pca_values[0]/(pca_values[0]+pca_values[1]+pca_values[2]);
  lambda1 = pca_values[1]/(pca_values[0]+pca_values[1]+pca_values[2]);
  lambda2 = pca_values[2]/(pca_values[0]+pca_values[1]+pca_values[2]);
  pca_vector = cpca.getEigenVectors(); // Normalized eigenvectors
  cout << "[Filtering Utils] Normalized Alignability Eigenvalues [max,...,min]: " << "\n" << pca_values << endl;

  // Features computed from eigenvalues
//  float linearity = (lambda0-lambda1)/lambda0;
//  float planarity = (lambda1-lambda2)/lambda0;
  float scattering = lambda2/lambda0;
//  float omnivariance = pow((lambda0*lambda1*lambda2),1/3.);
//  float anisotropy = (lambda0-lambda2)/lambda0;
//  float eigentropy = -(lambda0*log(lambda0))
//                     -(lambda1*log(lambda1))
//                     -(lambda2*log(lambda2));

//  cout << "----------[Filtering Utils] PCA ANALYSIS OF EGI----------------" << endl;
//  cout << "Linearity: " << "\t" << linearity << endl;
//  cout << "Planarity: " << "\t" << planarity << endl;
//  cout << "Scattering: " << "\t" << scattering << endl;
//  cout << "Omnivariance: " << "\t" << omnivariance << endl;
//  cout << "Anisotropy: " << "\t" << anisotropy << endl;
//  cout << "Eigentropy: " << "\t" << eigentropy << endl;
//  cout << "---------------------------------------------------------------" << endl;

  alignability = scattering*100.0;

  // Visualization: return eigenvalues frame
  eigenvectors->width = 3;
  eigenvectors->height = 1;
  eigenvectors->points.resize(eigenvectors->width * eigenvectors->height);
  Eigen::Vector4f cloud_centroid;
  compute3DCentroid(*cloudA_planes, cloud_centroid);
  for (int i = 0; i < eigenvectors->size(); i++) {
    eigenvectors->points[i].x = cloud_centroid[0];
    eigenvectors->points[i].y = cloud_centroid[1];
    eigenvectors->points[i].z = cloud_centroid[2];
    eigenvectors->points[i].r = 0.0;
    eigenvectors->points[i].g = 0.0;
    eigenvectors->points[i].b = 0.0;
    eigenvectors->points[i].normal_x = pca_vector(0,i);
    eigenvectors->points[i].normal_y = pca_vector(1,i);
    eigenvectors->points[i].normal_z = pca_vector(2,i);
    if(i==0)
      eigenvectors->points[i].r = 255.0;
    else if(i==1)
      eigenvectors->points[i].g = 255.0;
    else
      eigenvectors->points[i].b = 255.0;
  }

//  pcl::PCDWriter writer;
//  writer.write<pcl::PointXYZRGBNormal> ("matching_planesA.pcd", *cloudA_planes, false);
//  writer.write<pcl::PointXYZRGBNormal> ("matching_planesB.pcd", *cloudB_planes, false);
  return alignability;
}

void computeNormalsCentroid(pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud, Eigen::Vector3f& centroid)
{
  float sum_x = 0; float sum_y = 0; float sum_z = 0;
  for (int k = 0; k < cloud.size(); k++)
  {
    sum_x = sum_x + cloud.points[k].normal_x;
    sum_y = sum_y + cloud.points[k].normal_y;
    sum_z = sum_z + cloud.points[k].normal_z;
  }
  centroid[0] = sum_x/cloud.size();
  centroid[1] = sum_y/cloud.size();
  centroid[2] = sum_z/cloud.size();
}

float overlapBoxFilter(pcl::PointCloud<pcl::PointXYZRGBNormal>& planeA, pcl::PointCloud<pcl::PointXYZRGBNormal>& planeB)
{
  // Build bounding box around plane B
  Eigen::Vector4f min_ptB, max_ptB;
  pcl::getMinMax3D (planeB, min_ptB, max_ptB);
  // Enlarge boundaries to become 3 times the distance along that direction
  for (int i=0; i<3; i++)
  {
    float dist = abs(min_ptB[i]-max_ptB[i]);
    min_ptB[i] = min_ptB[i] - dist;
    max_ptB[i] = max_ptB[i] + dist;
  }
  // and select points from A which belong to box
  std::vector<int> ind_pointsA_in_boxB;
  pcl::getPointsInBox (planeA, min_ptB, max_ptB, ind_pointsA_in_boxB);

  // Build bounding box around plane A
  Eigen::Vector4f min_ptA, max_ptA;
  pcl::getMinMax3D (planeA, min_ptA, max_ptA);

  // Enlarge boundaries to become 3 times the distance along that direction
  for (int i=0; i<3; i++)
  {
    float dist = abs(min_ptA[i]-max_ptA[i]);
    min_ptA[i] = min_ptA[i] - dist;
    max_ptA[i] = max_ptA[i] + dist;
  }
  // and select points from B which belong to box
  std::vector<int> ind_pointsB_in_boxA;
  pcl::getPointsInBox (planeB, min_ptA, max_ptA, ind_pointsB_in_boxA);
//  pcl::PointCloud<pcl::PointXYZRGBNormal> pointsB_in_boxA (planeB, ind_pointsB_in_boxA);
//
//  // Create cloud for Visualization
//  pcl::PointCloud<pcl::PointXYZRGBNormal> minMaxA;
//  pcl::PointXYZRGBNormal minA, maxA;
//  float rgb_white = 256.0;
//  float r_red = 256.0, g_red = 0.0, b_red = 0.0;
//  float r_green = 0.0, g_green = 256.0, b_green = 0.0;
//  minA.x = min_ptA[0];
//  minA.y = min_ptA[1];
//  minA.z = min_ptA[2];
//  minA.r = rgb_white;
//  minA.g = rgb_white;
//  minA.b = rgb_white;
//  minMaxA.push_back(minA);
//  maxA.x = max_ptA[0];
//  maxA.y = max_ptA[1];
//  maxA.z = max_ptA[2];
//  maxA.r = r_red;
//  maxA.g = g_red;
//  maxA.b = b_red;
//  minMaxA.push_back(maxA);
//  for (size_t i_point = 0; i_point < pointsB_in_boxA.points.size (); i_point++)
//  {
//    pointsB_in_boxA.points[i_point].r = r_green;
//    pointsB_in_boxA.points[i_point].g = g_green;
//    pointsB_in_boxA.points[i_point].b = b_green;
//    minMaxA.push_back(pointsB_in_boxA.points[i_point]);
//  }
//  pcl::PCDWriter writer;
//  writer.write<pcl::PointXYZRGBNormal> ("minMaxA.pcd", minMaxA, false);

  //Compute overlap parameter dependent on percentage of accepted points per cloud
  float perc_accepted_A, perc_accepted_B;
  perc_accepted_A = (float)(ind_pointsA_in_boxB.size()) / (float)(planeA.size());
  perc_accepted_B = (float)(ind_pointsB_in_boxA.size()) / (float)(planeB.size());
//  cout << "Points left in cloudA: " << perc_accepted_A*100.0 << "%" << endl;
//  cout << "Points left in cloudB: " << perc_accepted_B*100.0 << "%" << endl;

  float overlap = perc_accepted_A * perc_accepted_B;
//  cout << "Box Overlap: " << overlap*100.0 << "%" << endl;

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
  cout << "[Filtering Utils] Normalized Prediction Eigenvalues [R,P,Y,X,Y,Z]:" << endl << system_lambdas << endl;

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
