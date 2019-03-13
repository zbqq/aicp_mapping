// aicp_create_cube_cloud

#include <iostream>

//eigen
//#include <Eigen/Dense>

//pcl
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>

int main()
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  float min_corner = -2.00;
  float max_corner = 2.00;
  float step = 0.05;

  // bottom
  for (float i = min_corner; i < max_corner; i += step) {
    for (float j = min_corner; j < max_corner; j += step) {
      pcl::PointXYZ surface_point;
      surface_point.x = i;
      surface_point.y = j;
      surface_point.z = min_corner;
      cloud.points.push_back(surface_point);
    }
  }
  // top
  for (float i = min_corner; i < max_corner; i += step) {
    for (float j = min_corner; j < max_corner; j += step) {
      pcl::PointXYZ surface_point;
      surface_point.x = i;
      surface_point.y = j;
      surface_point.z = max_corner;
      cloud.points.push_back(surface_point);
    }
  }
  // side A
  for (float i = min_corner; i < max_corner; i += step) {
    for (float j = min_corner; j < max_corner; j += step) {
      pcl::PointXYZ surface_point;
      surface_point.x = min_corner;
      surface_point.y = i;
      surface_point.z = j;
      cloud.points.push_back(surface_point);
    }
  }
  // side B
  for (float i = min_corner; i < max_corner; i += step) {
    for (float j = min_corner; j < max_corner; j += step) {
      pcl::PointXYZ surface_point;
      surface_point.x = max_corner;
      surface_point.y = i;
      surface_point.z = j;
      cloud.points.push_back(surface_point);
    }
  }
  // side C
  for (float i = min_corner; i < max_corner; i += step) {
    for (float j = min_corner; j < max_corner; j += step) {
      pcl::PointXYZ surface_point;
      surface_point.x = i;
      surface_point.y = min_corner;
      surface_point.z = j;
      cloud.points.push_back(surface_point);
    }
  }
  // side D
  for (float i = min_corner; i < max_corner; i += step) {
    for (float j = min_corner; j < max_corner; j += step) {
      pcl::PointXYZ surface_point;
      surface_point.x = i;
      surface_point.y = max_corner;
      surface_point.z = j;
      cloud.points.push_back(surface_point);
    }
  }
  cloud.width = cloud.points.size();
  cloud.height = 1;

  // save cloud to disk
  pcl::PCDWriter writer;
  std::stringstream ss;
  ss << "cube_cloud.pcd";
  writer.write<pcl::PointXYZ> (ss.str (), cloud, false);

  return (0);
}
