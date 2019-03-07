// pcl_remove_ground -i <input_cloud> -o <output_cloud>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>

// args
#include <ConciseArgs>

using namespace std;

struct CommandLineConfig
{
  ///The file to read from.
  string infile;
  
  ///The file to output to.
  string outfile;
};

int
main (int argc, char** argv)
{
  CommandLineConfig cl_cfg;

  ConciseArgs parser(argc, argv, "project_to_parametric_model");
  parser.add(cl_cfg.infile, "i", "infile", "Input file location");
  parser.add(cl_cfg.outfile, "o", "outfile", "Output file location");
  parser.parse();
  
  if (cl_cfg.infile.empty() || cl_cfg.outfile.empty())
  {
    cerr << "Input and output files required." << endl;
    exit(EXIT_FAILURE);
  }

  // Load input file into a PointCloud<T> with an appropriate type
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI> ());

  pcl::io::loadPCDFile (cl_cfg.infile, *cloud);

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointIndicesPtr ground (new pcl::PointIndices);

  std::cerr << "Cloud before filtering: " << std::endl;
  std::cerr << *cloud << std::endl;

  // Create the filtering object
  pcl::ProgressiveMorphologicalFilter<pcl::PointXYZI> pmf;
  pmf.setInputCloud (cloud);
  pmf.setMaxWindowSize (1);
  pmf.setSlope (0.1f);
  pmf.setInitialDistance (0.1f);
  pmf.setMaxDistance (2.0f);
  pmf.extract (ground->indices);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud (cloud);
  extract.setIndices (ground);
  extract.filter (*cloud_filtered);

  std::cerr << "Ground cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;

  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZI> ("sampled_ground.pcd", *cloud_filtered, false);

  // Extract non-ground returns
  extract.setNegative (true);
  extract.filter (*cloud_filtered);

  std::cerr << "Object cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;

  writer.write<pcl::PointXYZI> (cl_cfg.outfile, *cloud_filtered, false);

  return (0);
}
