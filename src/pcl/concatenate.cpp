#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h> // console::parse_file_extension_argument

int
  main (int argc, char** argv)
{
  // load a pointcloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_a_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_b_ptr (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PointCloud<pcl::PointXYZ>& cloud_a = *cloud_a_ptr;
  pcl::PointCloud<pcl::PointXYZ>& cloud_b = *cloud_b_ptr;
  std::vector<int> pcd_filename_indices = pcl::console::parse_file_extension_argument (argc, argv, "pcd");
  if (!pcd_filename_indices.empty())
  {
    std::string filename = argv[pcd_filename_indices[0]];
    if (pcl::io::loadPCDFile (filename, cloud_a) == -1)
    {
      std::cerr << "Was not able to open file \""<<filename<<"\"." << std::endl;
      return EXIT_SUCCESS;
    }
    std::string filename2 = argv[pcd_filename_indices[1]];
    if (pcl::io::loadPCDFile (filename2, cloud_b) == -1)
    {
      std::cerr << "Was not able to open file \""<<filename2<<"\"." << std::endl;
      return EXIT_SUCCESS;
    }

  } else {
    std::cout << "Please specify point cloud file(s)" << std::endl;
    return EXIT_SUCCESS;
  }

  // concatination goes here
  pcl::PointCloud<pcl::PointXYZ> point_cloud_assembled;

  point_cloud_assembled  = cloud_a;
  point_cloud_assembled += cloud_b;

  // save cloud to disk
  pcl::PCDWriter writer;
  std::stringstream ss;
  ss << "assembled_cloud.pcd";
  writer.write<pcl::PointXYZ> (ss.str (), point_cloud_assembled, false);

  return (0);
}
