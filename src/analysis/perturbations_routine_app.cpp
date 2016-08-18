#include <lcm/lcm-cpp.hpp>
#include <ConciseArgs>

#include <icp-registration/cloud_accumulate.hpp>
#include <icp-registration/icp_3Dreg_and_plot.hpp>
#include <icp-registration/icp_utils.h>
#include <icp-registration/vtkUtils.h>

#include <drawingUtils/drawingUtils.hpp>
#include <filteringUtils/filteringUtils.hpp>

/**
  * Code to test ICP sensitivity to input: initial perturbations.
  */
  // Input: two aligned clouds (overlap >> 70%)
  // Output: file with alignement results given x-y-yaw perturbation
  //         each line: x, y, yaw, transl_error, rot_error, failed_bool

struct CommandLineConfig
{
  std::string algorithm;
};

class App{
  public:
    App(boost::shared_ptr<lcm::LCM> &lcm_, const CommandLineConfig& cl_cfg_);

    ~App(){
    }

    // Basin of convergence
    float square_size_; //in meters
    float square_step_; //in meters

    void doRoutine(string ref_name, string read_name, string out_file_name);

  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    const CommandLineConfig cl_cfg_;

    // Local reference initialized to identity
    Eigen::Isometry3d local_;

    // Initialize ICP
    RegistrationConfig reg_cfg_;
    // Temporary config file for ICP chain: copied and trimmed ratio replaced
    string tmpConfigName_;

    // Error functions
    float errorTransl(PM::TransformationParameters& deltaTransl);
    float errorRot(PM::TransformationParameters& deltaR);
};

App::App(boost::shared_ptr<lcm::LCM> &lcm_, const CommandLineConfig& cl_cfg_) : lcm_(lcm_), cl_cfg_(cl_cfg_){
  // size = dimensions of the squared distribution of perturbations
  //        around the 0,0 (alignment pose) of the clouds.
  // step = distance between samples in the square (in meters).
  square_size_ = 2.0;
  square_step_ = 0.1;

  local_ = Eigen::Isometry3d::Identity();

  // File used to update config file for ICP chain
  tmpConfigName_.append(getenv("DRC_BASE"));
  tmpConfigName_.append("/software/perception/registration/filters_config/icp_autotuned_default.yaml");
}

void App::doRoutine(string ref_name, string read_name, string out_file_name)
{
  // Load aligned clouds (with overlap >> 70%)
  float overlap_ = 100.0;
  DP reference = DP::load(ref_name);
  DP reading = DP::load(read_name);

  // PRE-FILTERING using filteringUtils
  if (cl_cfg_.algorithm == "aicp")
  {
    //planeModelSegmentationFilter(reference);
    //planeModelSegmentationFilter(reading);
    regionGrowingPlaneSegmentationFilter(reference);
    regionGrowingPlaneSegmentationFilter(reading);
  }

  // ............do registration.............
  // First ICP loop
  string configName1;
  configName1.append(getenv("DRC_BASE"));
  if (cl_cfg_.algorithm == "icp")
    configName1.append("/software/perception/registration/filters_config/Chen91_pt2plane.yaml");
  else if (cl_cfg_.algorithm == "aicp")
    configName1.append("/software/perception/registration/filters_config/icp_autotuned.yaml");

  // To director
  drawPointCloudCollections(lcm_, 0, local_, reference, 1);
  drawPointCloudCollections(lcm_, 1, local_, reading, 1);

  Registration* registr;
  int index = square_size_/square_step_;
  int iteration = 0;
  if (index % 2 != 0)
  {
    index = index - 1;
  }
  for (int i = (index/2); i >= (-index/2); i --)
  {
    float y_perturbation = i * square_step_;

    for (int j = (-index/2); j <= (index/2); j ++)
    {
      float x_perturbation = j * square_step_;

      for (float k = -90; k <= 90; k=k+10)
      {
        float yaw_perturbation = k;

        std::stringstream init_transf_string;
        init_transf_string << to_string(x_perturbation);
        init_transf_string << ",";
        init_transf_string << to_string(y_perturbation);
        init_transf_string << ",";
        init_transf_string << to_string(yaw_perturbation);

        //cout << "i: " << i << " j: " << j << "  initT: " << init_transf_string.str() << endl;

        // Load initial transform
        reg_cfg_.initTrans_.clear();
        reg_cfg_.initTrans_.append(init_transf_string.str());

        // ICP chain
        registr = new Registration(reg_cfg_);

        // Auto-tune ICP chain (quantile for the Trimmed Outlier Filter)
        float current_ratio = overlap_/100.0;
        if (current_ratio < 0.25)
          current_ratio = 0.25;
        else if (current_ratio > 0.70)
          current_ratio = 0.70;

        if (cl_cfg_.algorithm == "aicp")
          replaceRatioConfigFile(tmpConfigName_, configName1, current_ratio);

        registr->setConfigFile(configName1);
        registr->getICPTransform(reading, reference);
        PM::TransformationParameters deltaT = registr->getTransform();
        cout << "3D Delta Transformation" << endl << deltaT << endl;

        PM::ICP icp = registr->getIcp();
        DP readFiltered = icp.getReadingFiltered();

        // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        // To file: DEBUG
        /*std::stringstream vtk_fname;
        vtk_fname << "afterInitICP_";
        vtk_fname << to_string(iteration);
        vtk_fname << ".vtk";
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr initializedReadingPtr = registr->getCloud(3);
        pcl::PointCloud<pcl::PointXYZRGB> initializedReading = *initializedReadingPtr;
        DP dp_cloud;
        fromPCLToDataPoints(dp_cloud, initializedReading);
        savePointCloudVTK(vtk_fname.str().c_str(), dp_cloud);
        // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        // To file: DEBUG
        std::stringstream vtk_fname2;
        vtk_fname2 << "outputICP_";
        vtk_fname2 << to_string(iteration);
        vtk_fname2 << ".vtk";
        DP output = registr->getDataOut();
        savePointCloudVTK(vtk_fname2.str().c_str(), output);*/
        // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        // Compute 3D translation error
        PM::TransformationParameters deltaTransl = deltaT.block(0,3,3,1);
        float errTransl = errorTransl(deltaTransl);
        cout << "3D Translation Error (meters): " << errTransl << endl;
        // Compute 3D rotation error
        PM::TransformationParameters deltaR = deltaT.block(0,0,3,3);
        float errRot = errorRot(deltaR);
        cout << "3D Rotation Error (degrees): " << errRot << endl;

        bool alignment_failed = false;
        if (errTransl > 0.06 || errRot > 2)
          alignment_failed = true;
        cout << "FAILED? " << alignment_failed << endl;

        // Write result to file
        PM::Matrix line_to_file(1,6);
        // LINE: x, y, yaw, transl_error, rot_error, failed_bool
        line_to_file << x_perturbation, y_perturbation, yaw_perturbation, errTransl, errRot, alignment_failed;
        writeLineToFile(line_to_file, "perturbation_results.txt", iteration);

        delete registr;
        iteration ++;
      }
    }
  }
}

int main(int argc, char **argv){
  CommandLineConfig cl_cfg;
  cl_cfg.algorithm = "aicp";

  ConciseArgs parser(argc, argv, "simple-fusion");
  parser.add(cl_cfg.algorithm, "a", "algorithm", "AICP or ICP? (i.e. aicp or icp)");
  parser.parse();

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  App* app= new App(lcm, cl_cfg);

  // Set names
  string refName, readName, outFileName;
  refName.append(getenv("HOME"));
  //refName.append("/logs/valkyrie/perturbation_test/accum_reference.vtk");
  refName.append("/logs/valkyrie/perturbation_test/cloud_1.vtk");
  readName.append(getenv("HOME"));
  //readName.append("/logs/valkyrie/perturbation_test/accum_reading.vtk");
  readName.append("/logs/valkyrie/perturbation_test/cloud_1.vtk");
  outFileName.append(getenv("HOME"));
  outFileName.append("/logs/valkyrie/perturbation_test/transformation_errors.txt");

  app->square_size_ = 2.0;
  app->square_step_ = 0.1;

  app->doRoutine(refName, readName, outFileName);
}

float App::errorTransl(PM::TransformationParameters& deltaTransl)
{
  return sqrt( pow(deltaTransl(0,0),2) + pow(deltaTransl(1,0),2) + pow(deltaTransl(2,0),2) );
}

float App::errorRot(PM::TransformationParameters& deltaR)
{
  float trace = deltaR(0,0) + deltaR(1,1) + deltaR(2,2);

  // 3D rotation error in degrees
  return ( acos((trace-1.0)/2.0) * 180.0 / M_PI );
}