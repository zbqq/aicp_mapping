#include "cloudIO.h"

int savePlanarCloudCSV (const std::string &file_name, const pcl::PCLPointCloud2 &cloud)
{
  if (cloud.data.empty ())
  {
    PCL_ERROR ("[icp_io_utils:savePlanarCloudCSV] Input point cloud has no data!\n");
    return (-1);
  }
 
  // Open file
  std::ofstream fs;
  //fs.precision (precision);
  fs.open (file_name.c_str ());

  unsigned int nr_points  = cloud.width * cloud.height;
  unsigned int point_size = static_cast<unsigned int> (cloud.data.size () / nr_points);

  // Iterate through the points
  for (unsigned int i = 0; i < nr_points; ++i)
  {
    int xy = 0;
    for (size_t d = 0; d < cloud.fields.size (); ++d)
    {
      int count = cloud.fields[d].count;
      if (count == 0)
        count = 1;          // we simply cannot tolerate 0 counts (coming from older converter code)
      int c = 0;
      if ((cloud.fields[d].datatype == pcl::PCLPointField::FLOAT32) && (
       	cloud.fields[d].name == "x" || cloud.fields[d].name == "y"))
      {
        float value;
        memcpy (&value, &cloud.data[i * point_size + cloud.fields[d].offset + c * sizeof (float)], sizeof (float));
        fs << value;
        if (++xy == 2)
          break;
      }
      fs << " , ";
	}
    if (xy != 2)
    {
	  PCL_ERROR ("[icp_io_utils:savePlanarCloudCSV] Input point cloud has no XY data!\n");
	  return (-2);
	}
	fs << std::endl;
  }

  // Close file
  fs.close ();
  return (0); 
}

void savePointCloudPCLwithPose(const std::string file_name, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, Eigen::Isometry3d sensor_pose)
{
  cloud->sensor_origin_.x() = sensor_pose.translation().x();
  cloud->sensor_origin_.y() = sensor_pose.translation().y();
  cloud->sensor_origin_.z() = sensor_pose.translation().z();
  Eigen::Quaterniond pose_quat(sensor_pose.rotation());
  cloud->sensor_orientation_.x() =  pose_quat.x();
  cloud->sensor_orientation_.y() =  pose_quat.y();
  cloud->sensor_orientation_.z() =  pose_quat.z();
  cloud->sensor_orientation_.w() =  pose_quat.w();

  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZRGB> (file_name, *cloud, false);
}

void fromDataPointsToPCL(DP &cloud_in, pcl::PointCloud<pcl::PointXYZ> &cloud_out)
{
  cloud_out.points.resize(cloud_in.getNbPoints());
  for (int i = 0; i < cloud_in.getNbPoints(); i++) {
    cloud_out.points[i].x = (cloud_in.features.col(i))[0];
    cloud_out.points[i].y = (cloud_in.features.col(i))[1];
    cloud_out.points[i].z = (cloud_in.features.col(i))[2];
    //cout << "i=" << i << " " << cloud_out.points[i].x << " " << cloud_out.points[i].y << " " << cloud_out.points[i].z << endl;
  }
  cloud_out.width = cloud_out.points.size();
  cloud_out.height = 1;
}

void fromPCLToDataPoints(DP &cloud_out, pcl::PointCloud<pcl::PointXYZ> &cloud_in)
{
  int pointCount = cloud_in.width;

  PM::Matrix features(4, pointCount);

  for (int p = 0; p < pointCount; ++p)
  {
    features(0, p) = cloud_in.points[p].x;
    features(1, p) = cloud_in.points[p].y;
    features(2, p) = cloud_in.points[p].z;
    features(3, p) = 1.0;
  }
  cloud_out.addFeature("x", features.row(0));
  cloud_out.addFeature("y", features.row(1));
  cloud_out.addFeature("z", features.row(2));
  cloud_out.addFeature("pad", features.row(3));
}

// TODO: These methods should deal with rgb field as well
void fromDataPointsToPCL(DP &cloud_in, pcl::PointCloud<pcl::PointXYZRGB> &cloud_out)
{
  cloud_out.points.resize(cloud_in.getNbPoints());
  for (int i = 0; i < cloud_in.getNbPoints(); i++) {
    cloud_out.points[i].x = (cloud_in.features.col(i))[0];
    cloud_out.points[i].y = (cloud_in.features.col(i))[1];
    cloud_out.points[i].z = (cloud_in.features.col(i))[2];
    //cout << "i=" << i << " " << cloud_out.points[i].x << " " << cloud_out.points[i].y << " " << cloud_out.points[i].z << endl;
  }
  cloud_out.width = cloud_out.points.size();
  cloud_out.height = 1;
}

void fromPCLToDataPoints(DP &cloud_out, pcl::PointCloud<pcl::PointXYZRGB> &cloud_in)
{
  int pointCount = cloud_in.width;

  PM::Matrix features(4, pointCount);

  for (int p = 0; p < pointCount; ++p)
  {
    features(0, p) = cloud_in.points[p].x;
    features(1, p) = cloud_in.points[p].y;
    features(2, p) = cloud_in.points[p].z;
    features(3, p) = 1.0;
  }
  cloud_out.addFeature("x", features.row(0));
  cloud_out.addFeature("y", features.row(1));
  cloud_out.addFeature("z", features.row(2));
  cloud_out.addFeature("pad", features.row(3));
}
