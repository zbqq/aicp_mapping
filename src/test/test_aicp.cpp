// Demo program for testing AICP:
//  1. Reading two unfiltered point clouds from file (with sensor origin information)
//  2. Running overlap analysis
//  3. Pre-filtering using regionGrowingPlaneSegmentationFilter
//  4. Running ICP alignment
//  5. Writing result to file and console
//  6. Compare file with expected result

// Test data at: "HOME"/drc-testing-data/aicp-data
// Command: test-aicp

// Get path to AICP base
#ifdef CONFDIR
  # define AICP_BASE CONFDIR
#else
  # define AICP_BASE "undefined"
#endif

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <lcm/lcm-cpp.hpp>

#include <icp-registration/icp_3Dreg_and_plot.hpp>
#include <icp-registration/icp_utils.h>

#include "filteringUtils/filteringUtils.hpp"
#include "drawingUtils/drawingUtils.hpp"

void doRegistration(Registration* reg, float overlap, DP &reference, DP &reading, DP &output, PM::TransformationParameters &T);
void getPoseFromPointCloud(pcl::PointCloud<pcl::PointXYZRGB>& cloud, Eigen::Isometry3d& pose);
void writeResultToFile(string out_file, int line_number, float filtered_cloud_size, 
                       float standard_overlap, PM::TransformationParameters &T);
float compareResultFiles(string file1, string file2);

int main (int argc, char** argv)
{
  const char *homedir;
  if ((homedir = getenv("HOME")) == NULL) {
    homedir = getpwuid(getuid())->pw_dir;
  }

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  int number_test_clouds = 16;

  // Standard Overlap parameter
  float standard_overlap_ = -1;
  float angular_view_ = 270.0;
  float max_range_ = 15.0;
  // Octree-based Overlap parameter
  /*vector<float> octrees_overlap_ = {-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0};
  bool set_reference_octree_ = TRUE;
  OverlapFromOctreesConfig ofo_cfg_;  
  ofo_cfg_.octomap_resolution = 0.2;
  OverlapFromOctrees* ofo_ = new OverlapFromOctrees(ofo_cfg_);*/

  // ICP
  RegistrationConfig reg_cfg;
  reg_cfg.initTrans_.clear();
  reg_cfg.initTrans_.append("0,0,0"); 
  Registration* registr_ = new Registration(reg_cfg);

  // Expected result file
  std::stringstream expected_file;
  expected_file << homedir;
  expected_file << "/drc-testing-data/aicp-data/test_aicp_expected.txt";
  // Output file
  std::stringstream out_file;
  out_file << homedir;
  out_file << "/drc-testing-data/aicp-data/test_aicp_output.txt";
  std::ofstream ofs;
  ofs.open(out_file.str().c_str(), std::ofstream::out | std::ofstream::trunc);
  ofs.close();

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr reference (new pcl::PointCloud<pcl::PointXYZRGB>);
  DP dp_ref;
  DP ref_filters;
  Eigen::Isometry3d ref_pose;

  for (int i = 0; i < number_test_clouds; i ++)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    DP dp_read;
  
    std::stringstream pcl_name;
    pcl_name << homedir;
    pcl_name << "/drc-testing-data/aicp-data/cloud_";
    pcl_name << std::to_string(i);
    pcl_name << ".pcd";

    if (i == 0) // load reference cloud from file
    {
      if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (pcl_name.str().c_str(), *reference) == -1)
      {
        PCL_ERROR ("Error loading PCD file.\n");
        return (-1);
      }
      std::cout << "Loaded " << reference->width * reference->height << " data points from " << pcl_name.str().c_str() << std::endl;

      fromPCLToDataPoints(dp_ref, *reference);
      getPoseFromPointCloud(*reference, ref_pose);

      ref_filters = dp_ref;
    }
    else
    {
      // load i-th reading cloud from file
      if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (pcl_name.str().c_str(), *cloud) == -1)
      {
        PCL_ERROR ("Error loading PCD file.\n");
        return (-1);
      }      
      std::cout << "Loaded " << cloud->width * cloud->height << " data points from " << pcl_name.str().c_str() << std::endl;

      fromPCLToDataPoints(dp_read, *cloud);
      Eigen::Isometry3d read_pose;
      getPoseFromPointCloud(*cloud, read_pose);

      DP read_filters = dp_read;
      // Compute Standard Overlap ------------------
      standard_overlap_ = overlapFilter(ref_filters, read_filters, ref_pose, read_pose, max_range_, angular_view_);
      // Compute Octree-based Overlap --------------
      /*ColorOcTree* read_tree = new ColorOcTree(ofo_cfg_.octomap_resolution);
      if(set_reference_octree_)
      {
        octreeBasedOverlapFilter(ofo_, ref_filters, read_filters, ref_pose, read_pose, read_tree, octrees_overlap_);
        set_reference_octree_ = FALSE;
      }
      else
        octreeBasedOverlapFilter(ofo_, read_filters, read_pose, read_tree, octrees_overlap_);*/

      cout << "Standard Overlap: " << standard_overlap_ << "%" << endl;
      //cout << "Octree-based Overlap: " << octrees_overlap_[0] << "%" << endl;
      
      // Pre-filtering:
      //  1. Down-sampling
      //  2. Region growing planes extraction
      regionGrowingPlaneSegmentationFilter(dp_ref);
      regionGrowingPlaneSegmentationFilter(dp_read);

      // Clouds Alignment
      DP dp_out;
      PM::TransformationParameters T;
      doRegistration(registr_, standard_overlap_, dp_ref, dp_read, dp_out, T);

      // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      // Publish clouds to LCM
      Eigen::Isometry3d local = Eigen::Isometry3d::Identity();
      drawPointCloudCollections(lcm, 0, local, dp_ref, 1, "Reference");
      drawPointCloudCollections(lcm, 1, local, dp_read, 1, "Reading");
      drawPointCloudCollections(lcm, 2, local, dp_out, 1, "Output");
      // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      // Write results to file
      writeResultToFile(out_file.str().c_str(), i, dp_read.getNbPoints(), standard_overlap_, T);
      // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      cout << "\n" << "-------------------------------------------------------------------------------" << "\n \n";
    }
  }
  // Compare with expected results from file
  float match = compareResultFiles(expected_file.str().c_str(), out_file.str().c_str());
  cout << "Results Match: " << match << "%" << endl;

  return (0);
}

void doRegistration(Registration* reg, float overlap, DP &reference, DP &reading, DP &output, PM::TransformationParameters &T)
{
  // File used to update config file for ICP chain
  string tmp_config_file;
  tmp_config_file.append(AICP_BASE);
  tmp_config_file.append("/filters_config/icp_autotuned_default.yaml");
  string config_file;
  config_file.append(AICP_BASE);
  config_file.append("/filters_config/icp_autotuned.yaml");

  // Auto-tune ICP chain (quantile for the Trimmed Outlier Filter)
  float current_ratio = overlap/100.0;
  if (current_ratio < 0.25)
    current_ratio = 0.25;
  else if (current_ratio > 0.70)
    current_ratio = 0.70;

  replaceRatioConfigFile(tmp_config_file, config_file, current_ratio);

  reg->setConfigFile(config_file);
  reg->getICPTransform(reading, reference);
  T = reg->getTransform();
  cout << "3D Transformation:" << endl << T << "\n";

  PM::ICP icp = reg->getIcp();
  // Store output after pre-filtering and alignment
  output = reading;
  icp.transformations.apply(output, T);
}

void getPoseFromPointCloud(pcl::PointCloud<pcl::PointXYZRGB>& cloud, Eigen::Isometry3d& pose)
{
  pose.setIdentity();
  pose.translation() << cloud.sensor_origin_.x(), cloud.sensor_origin_.y(), cloud.sensor_origin_.z();
  Eigen::Quaterniond quat = Eigen::Quaterniond(cloud.sensor_orientation_.w(),
                                               cloud.sensor_orientation_.x(),
                                               cloud.sensor_orientation_.y(),
                                               cloud.sensor_orientation_.z());
  pose.rotate(quat);
}

void writeResultToFile(string out_file, int line_number, float filtered_cloud_size, 
                       float standard_overlap, PM::TransformationParameters &T)
{
  Eigen::MatrixXf line = Eigen::MatrixXf::Zero(1, 18);
  line << filtered_cloud_size, standard_overlap,
          T(0,0), T(0,1), T(0,2), T(0,3),
          T(1,0), T(1,1), T(1,2), T(1,3),
          T(2,0), T(2,1), T(2,2), T(2,3),
          T(3,0), T(3,1), T(3,2), T(3,3);
  writeLineToFile(line, out_file, line_number);
}

float compareResultFiles(string file1, string file2)
{
  ifstream file1_stream, file2_stream;

  file1_stream.open( file1.c_str(), ios::binary );
  file2_stream.open( file2.c_str(), ios::binary );

  if (!file1_stream)
  {
    cout << "Couldn't open file " << file1 << endl;
    return -1;
  }
  if (!file2_stream)
  {
    cout << "Couldn't open file " << file2 << endl;
    return -1;
  }

  // Compare number of lines
  int c1, c2;
  c1 = 0;
  c2 = 0;
  string str;
  while(!file1_stream.eof())
  {
    getline(file1_stream, str);
    c1++;
  }
  while(!file2_stream.eof())
  {
    getline(file2_stream, str);
    c2++;
  }

  file1_stream.clear();
  file1_stream.seekg(0,ios::beg);

  file2_stream.clear();
  file2_stream.seekg(0,ios::beg);

  if(c1 != c2)
  {
    cout << "Number of lines in files does not match." << "\n";
    cout << file1 << " has " << c1 << " lines, while "<< file2 << " has " << c2 << " lines." << "\n";
    return -1;
  }

  // Compare files line by line
  char string1[256], string2[256];
  int j = 0, error_count = 0;
  while(!file1_stream.eof())
  {
    file1_stream.getline(string1, 256);
    file2_stream.getline(string2, 256);
    j++;
    if(strcmp(string1, string2) != 0)
    {
      cout << j << "-th strings are not equal: " << endl;
      cout << " file1: " << string1 << endl;
      cout << " file2: " << string2 << endl;
      error_count++;
    }
  }

  return (float(j - error_count) * 100.0) / float(j); 
}