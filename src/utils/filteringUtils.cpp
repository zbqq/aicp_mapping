#include <iostream>
#include "filteringUtils.hpp"

using namespace Eigen;

/*void drawPointCloud(bot_lcmgl_t *lcmgl, pcl::PointCloud<pcl::PointXYZRGB>& pcl_cloud)
{
  vector<Eigen::Vector3f> point_cloud;
  int cloud_size = pcl_cloud.points.size();
  float n_supp_points = 70000.0; // max number of floats supported by lcm channel
  int step = round(cloud_size/n_supp_points);
  point_cloud.resize(n_supp_points);
  int j = 0;
  for (int point = 0; point < cloud_size; point=point+step)
  {
    if (j < point_cloud.size())
      point_cloud.at(j) << pcl_cloud.points[point].x, pcl_cloud.points[point].y, pcl_cloud.points[point].z; 
    j++;
  }
    
  drawPointCloud(lcmgl, point_cloud);
}*/


// Stopped implementing this filter. Is it useful? Points out of the 
// overlapping area should be rejected by the outlier filter.
// TO_DO: use this area to define an overlapping metric? 
/*void overlapFilter(Eigen::Isometry3d world_to_body_1, Eigen::Isometry3d world_to_body_2, 
                    DP &first_cloud, DP &second_cloud)
{
  MatrixXf x_values = (first_cloud.getFeatureCopyByName("x"));
  MatrixXf y_values = (first_cloud.getFeatureCopyByName("y"));
  MatrixXf z_values = (first_cloud.getFeatureCopyByName("z"));
  int cloud_size = first_cloud.getNbPoints();

  vector<Eigen::Vector3f> first_cloud_tmp;
  //point_cloud.resize(cloud_size);

  //Eigen::Isometry3f second_to_first;
  //second_to_first = world_to_body_2.inverse() * world_to_body_1;

  for (int i = 0; i < cloud_size; i=i+1)
  {
    Eigen::Vector3f point;
    point << x_values(i), y_values(i), z_values(i);

    // Transform i-th point from world to frame1
    point = world_to_body_1.inverse() * point;

  } 
//....................
}*/

