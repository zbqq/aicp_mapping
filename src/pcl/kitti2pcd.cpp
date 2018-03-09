// run kitti2pcd -h

#include <boost/program_options.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/point_operators.h>
#include <pcl/common/io.h>
#include <pcl/search/organized.h>
#include <pcl/search/octree.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/filters/voxel_grid.h>

#include <iostream>
#include <fstream>

// args
#include <ConciseArgs>

using namespace pcl;
using namespace std;

struct CommandLineConfig
{
	///The file to read from.
	string infile;
	
	///The file to output to.
	string outfile;
};

int main(int argc, char **argv){
  CommandLineConfig cl_cfg;

  ConciseArgs parser(argc, argv, "kitti2pcd");
  parser.add(cl_cfg.infile, "i", "infile", "Input file location");
  parser.add(cl_cfg.outfile, "o", "outfile", "Output file location");
  parser.parse();
  
  if (cl_cfg.infile.empty() || cl_cfg.outfile.empty())
  {
    cerr << "Input and output files required." << endl;
		exit(EXIT_FAILURE);
  }
/*	// Declare the supported options.
	po::options_description desc("Program options");
	desc.add_options()
		//Options
		("infile", po::value<string>(&infile)->required(), "the file to read a point cloud from")
		("outfile", po::value<string>(&outfile)->required(), "the file to write the DoN point cloud & normals to")
		;
	// Parse the command line
	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);

	// Print help
	if (vm.count("help"))
	{
		cout << desc << "\n";
		return false;
	}

	// Process options.
	po::notify(vm);*/

	// load point cloud
	fstream input(cl_cfg.infile.c_str(), ios::in | ios::binary);
	if(!input.good()){
		cerr << "Could not read file: " << cl_cfg.infile << endl;
		exit(EXIT_FAILURE);
	}
	input.seekg(0, ios::beg);

	pcl::PointCloud<PointXYZI>::Ptr points (new pcl::PointCloud<PointXYZI>);

	int i;
	for (i=0; input.good() && !input.eof(); i++) {
		PointXYZI point;
		input.read((char *) &point.x, 3*sizeof(float));
		input.read((char *) &point.intensity, sizeof(float));
		points->push_back(point);
	}
	input.close();

	cout << "Read KITTI point cloud with " << i << " points, writing to " << cl_cfg.outfile << endl;

    pcl::PCDWriter writer;

    // Save DoN features
    writer.write<PointXYZI> (cl_cfg.outfile, *points, false);
}

