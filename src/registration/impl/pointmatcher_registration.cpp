#include "pointmatcher_registration.hpp"

namespace aicp{

//PointmatcherRegistration::PointmatcherRegistration()  {}

PointmatcherRegistration::PointmatcherRegistration(const RegistrationParams& params) :
        params_(params) {
}

PointmatcherRegistration::~PointmatcherRegistration() {}

//Templates
void PointmatcherRegistration::registerClouds(const pcl::PointCloud<pcl::PointXYZ>& cloud_ref, const pcl::PointCloud<pcl::PointXYZ>& cloud_read, Eigen::Matrix4f &final_transform)
{
  fromPCLToDataPoints(reference_cloud_, cloud_ref);
  fromPCLToDataPoints(reading_cloud_, cloud_read);

  registerClouds(final_transform);
}

/*
void PointmatcherRegistration::registerClouds(const pcl::PointCloud<pcl::PointXYZRGB>& cloud_ref, const pcl::PointCloud<pcl::PointXYZRGB>& cloud_read, Eigen::Matrix4f &final_transform)
{
  fromPCLToDataPoints(reference_cloud_, cloud_ref);
  fromPCLToDataPoints(reading_cloud_, cloud_read);

  registerClouds(final_transform);
}
/*
void PointmatcherRegistration::registerClouds(const pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud_ref, const pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud_read, Eigen::Matrix4f &final_transform)
{
  fromPCLToDataPoints(reference_cloud_, cloud_ref);
  fromPCLToDataPoints(reading_cloud_, cloud_read);

  registerClouds(final_transform);
}*/

//Load and apply configuration
void PointmatcherRegistration::applyConfig()
{
  // ICP chain configuration: check if prefiltering required
  if (params_.pointmatcher.configFileName.empty())
  {
    // Set ICP chain to default
    icp_.setDefault();
  }
  else
  {
    // load YAML config
    ifstream ifs(params_.pointmatcher.configFileName.c_str());
    if (!ifs.good())
    {
      cerr << "Cannot open config file " << params_.pointmatcher.configFileName << endl;
      exit(1);
    }
    icp_.loadFromYaml(ifs);
    cerr << "Loaded pre-filtering chain from yaml..." << endl;
  }
}

//Load and apply initialization
void PointmatcherRegistration::applyInitialization()
{
  // Apply rigid initial transformation
  init_transform_ = parseTransformationDeg(params_.pointmatcher.initialTransform, 3);

  PM::Transformation* rigid_transform = PM::get().REG(Transformation).create("RigidTransformation");

  if (!rigid_transform->checkParameters(init_transform_)) {
    cerr << endl
      << "Initial transformation is not rigid, identity will be used." << endl;
    init_transform_ = PM::TransformationParameters::Identity(4, 4);
  }
  else
    cout << "Initialization: " << params_.pointmatcher.initialTransform << endl;

  initialized_reading_ = rigid_transform->compute(reading_cloud_, init_transform_);
}

//Registration: Compute transform which aligns reading cloud onto the reference cloud.
void PointmatcherRegistration::registerClouds(Eigen::Matrix4f &final_transform)
{
  int cloud_dimension = reference_cloud_.getEuclideanDim();
  
  if (!(cloud_dimension == 3))
  {
    cerr << "Invalid input point clouds dimension." << endl;
    exit(1);
  }

  //Params
  applyConfig();
  applyInitialization();

  // Compute the transformation
  PM::TransformationParameters T = icp_(reading_cloud_, reference_cloud_, init_transform_);
  //Ratio of how many points were used for error minimization (defined as TrimmedDistOutlierFilter ratio)
  cout << "Accepted matches (inliers): " << (icp_.errorMinimizer->getWeightedPointUsedRatio())*100 << " %" << endl;

  // Transform reading with T
  DP tmp_out_read_cloud_(reading_cloud_);
  icp_.transformations.apply(tmp_out_read_cloud_, T);

  out_read_cloud_ = tmp_out_read_cloud_;

  out_transform_ = T;
}

}
