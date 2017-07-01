#include "icp_3Dreg_and_plot.hpp"

RegistrationConfig::RegistrationConfig(){
  if ((homedir = getenv("HOME")) == NULL) {
    homedir = getpwuid(getuid())->pw_dir;
  }

  cloud_name_A.clear();
  cloud_name_A.clear();
  configFile3D_.clear();
  initTrans_.append("0,0,0");
}

Registration::Registration(boost::shared_ptr<lcm::LCM> &lcm_, const RegistrationConfig& reg_cfg_):
    lcm_(lcm_), reg_cfg_(reg_cfg_){
  init();
}

Registration::Registration(const RegistrationConfig& reg_cfg_):reg_cfg_(reg_cfg_){
  init();
}

void Registration::init()
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ref_ptr (new pcl::PointCloud<pcl::PointXYZRGB> ());
  reference_cloud_ = cloud_ref_ptr;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_ptr (new pcl::PointCloud<pcl::PointXYZRGB> ());
  input_cloud_ = cloud_in_ptr;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_trans_ptr (new pcl::PointCloud<pcl::PointXYZRGB> ());
  initialized_input_cloud_ = cloud_trans_ptr;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_intransf_ptr (new pcl::PointCloud<pcl::PointXYZRGB> ());
  transformed_input_cloud_ = cloud_intransf_ptr;
}

// Get transform after ICP alignment of reference and input cloud. Publish to LCM channel. 
void Registration::getICPTransform(DP &cloud_in, DP &cloud_ref)
{
  // Transform input clouds into a pcl PointXYZRGB and publish for visualization
  fromDataPointsToPCL(cloud_ref, *reference_cloud_);
  fromDataPointsToPCL(cloud_in, *input_cloud_);

  int cloudDimension = cloud_ref.getEuclideanDim();
  
  if (!(cloudDimension == 3)) 
  {
    cerr << "Invalid input point clouds dimension." << endl;
    exit(1);
  }

  // ICP chain configuration: check if prefiltering required
  if (reg_cfg_.configFile3D_.empty())
  {
    // See the implementation of setDefault() to create a custom ICP algorithm
    icp_.setDefault();
  }
  else
  {
    // load YAML config
    ifstream ifs(reg_cfg_.configFile3D_.c_str());
    if (!ifs.good())
    {
      cerr << "Cannot open config file " << reg_cfg_.configFile3D_ << endl; exit(1);
    }
    icp_.loadFromYaml(ifs);
    cerr << "Loaded pre-filtering chain from yaml..." << endl;
  }

  // Apply rigid transformation (just a "visually good" approximation of the transformation 
  // between ref and input clouds) to escape local minima
  initT_ = parseTransformationDeg(reg_cfg_.initTrans_, cloudDimension);

  PM::Transformation* rigidTrans;
  rigidTrans = PM::get().REG(Transformation).create("RigidTransformation");

  if (!rigidTrans->checkParameters(initT_)) {
    cerr << endl
      << "Initial transformation is not rigid, identity will be used." << endl;
    initT_ = PM::TransformationParameters::Identity(cloudDimension+1,cloudDimension+1);
  }
  else
    cout << "Initialization: " << reg_cfg_.initTrans_ << endl;

  // Compute the transformation to express input in ref
  PM::TransformationParameters T = icp_(cloud_in, cloud_ref, initT_);
  //Ratio of how many points were used for error minimization (defined as TrimmedDistOutlierFilter ratio)
  cout << "Accepted matches (inliers): " << (icp_.errorMinimizer->getWeightedPointUsedRatio())*100 << " %" << endl;

  // Transform input to express it in ref
  DP data_out(cloud_in);
  icp_.transformations.apply(data_out, T);

  out_cloud_ = data_out;

  // Store input after initialization and after final transformation
  fromDataPointsToPCL(out_cloud_, *transformed_input_cloud_);
  DP initializedInput = rigidTrans->compute(cloud_in, initT_);
  fromDataPointsToPCL(initializedInput, *initialized_input_cloud_);

  out_transform_ = T;
}

void Registration::publishCloud(pronto_vis* vis, int cloud_id, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  Isometry3dTime null_T = Isometry3dTime(1, Eigen::Isometry3d::Identity());
  vis->pose_to_lcm_from_list(60000, null_T);
  vis->ptcld_to_lcm_from_list(cloud_id, *cloud, 1, 1);
}



