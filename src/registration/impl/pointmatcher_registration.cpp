#include "pointmatcher_registration.hpp"

namespace aicp{

  PointmatcherRegistration::PointmatcherRegistration()  {}

  PointmatcherRegistration::PointmatcherRegistration(const RegistrationParams& params) :
          params_(params) {
  }

  PointmatcherRegistration::~PointmatcherRegistration() {}

  //Templates
  void PointmatcherRegistration::registerClouds(pcl::PointCloud<pcl::PointXYZ>& cloud_ref, pcl::PointCloud<pcl::PointXYZ>& cloud_read, Eigen::Matrix4f &final_transform, vector<float>& failure_prediction_factors)
  {
    DP empty;
    reference_cloud_ = empty;
    reading_cloud_ = empty;
    fromPCLToDataPoints(reference_cloud_, cloud_ref);
    fromPCLToDataPoints(reading_cloud_, cloud_read);

    return registerClouds(final_transform, failure_prediction_factors);
  }

  void PointmatcherRegistration::registerClouds(pcl::PointCloud<pcl::PointXYZRGB>& cloud_ref, pcl::PointCloud<pcl::PointXYZRGB>& cloud_read, Eigen::Matrix4f &final_transform, vector<float>& failure_prediction_factors)
  {
    DP empty;
    reference_cloud_ = empty;
    reading_cloud_ = empty;
    fromPCLToDataPoints(reference_cloud_, cloud_ref);
    fromPCLToDataPoints(reading_cloud_, cloud_read);

    return registerClouds(final_transform, failure_prediction_factors);
  }

  void PointmatcherRegistration::registerClouds(pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud_ref, pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud_read, Eigen::Matrix4f &final_transform, vector<float>& failure_prediction_factors)
  {
//    DP empty;
//    reference_cloud_ = empty;
//    reading_cloud_ = empty;
//    fromPCLToDataPoints(reference_cloud_, cloud_ref);
//    fromPCLToDataPoints(reading_cloud_, cloud_read);

//    return registerClouds(final_transform, failure_prediction_factors);
  }

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
        cerr << "[Pointmatcher] Cannot open config file " << params_.pointmatcher.configFileName << endl;
        exit(1);
      }
      icp_.loadFromYaml(ifs);
      cerr << "[Pointmatcher] Loaded pre-filtering chain from yaml..." << endl;
    }
  }

  //Load initialization
  PM::TransformationParameters PointmatcherRegistration::applyInitialization()
  {
    PM::TransformationParameters init_transform = parseTransformationDeg(params_.pointmatcher.initialTransform, 3);

    PM::Transformation* rigid_transform = PM::get().REG(Transformation).create("RigidTransformation");

    if (!rigid_transform->checkParameters(init_transform)) {
      cerr << endl
        << "[Pointmatcher] Initial transformation is not rigid, identity will be used." << endl;
      init_transform = PM::TransformationParameters::Identity(4, 4);
    }
    else
      cout << "[Pointmatcher] Initialization: " << params_.pointmatcher.initialTransform << endl;

    // Manually transform reading to init position for visualization
    initialized_reading_ = rigid_transform->compute(reading_cloud_, init_transform);

    return init_transform;
  }

  //Registration: Compute transform which aligns reading cloud onto the reference cloud.
  void PointmatcherRegistration::registerClouds(Eigen::Matrix4f &final_transform, vector<float>& failure_prediction_factors)
  {
    int cloud_dimension = reference_cloud_.getEuclideanDim();

    if (!(cloud_dimension == 3))
    {
      cerr << "[Pointmatcher] Invalid input point clouds dimension." << endl;
      exit(1);
    }

    // Params
    applyConfig();

    // Compute the transformation
    PM::TransformationParameters T;
    PM::TransformationParameters init_transform = PM::TransformationParameters::Identity(4, 4);
    if (!params_.pointmatcher.initialTransform.empty())
      init_transform = applyInitialization();

    T = icp_(reading_cloud_, reference_cloud_, init_transform);

    //Ratio of how many points were used for error minimization (defined as TrimmedDistOutlierFilter ratio)
    cout << "[Pointmatcher] Accepted matches (inliers): " << (icp_.errorMinimizer->getWeightedPointUsedRatio())*100 << " %" << endl;

//    // simalpha: TO UPGRADE
//    // Get System Covariance:
//    Eigen::MatrixXf system_covariance = (icp_.errorMinimizer->getSystemCovariance());
//    //cout << "[Pointmatcher] System Covariance (A^T * A): " << endl << system_covariance << endl;
//    registrationFailurePredictionFilter(system_covariance, failure_prediction_factors);
//    //cout << "[Pointmatcher] Failure Prediction Factor: " << endl << failure_prediction_factor << endl;

    // Transform reading with T
    DP tmp_out_read_cloud_(reading_cloud_);
    icp_.transformations.apply(tmp_out_read_cloud_, T);

    out_read_cloud_ = tmp_out_read_cloud_;

    final_transform = T; // initialization is already included

    /*===================================
    =               Errors              =
    ===================================*/
    if (params_.pointmatcher.printOutputStatistics){
      /* TODO review error function in pointmatcherUtils/icpMonitor.h
       *
      DP out = app->registr_->getDataOut();
      float hausDist = hausdorffDistance(ref, out);

      cout << "Hausdorff distance: " << hausDist << " m" << endl;

      PM::ICP icp = app->registr_->getIcp();
      float meanDist = pairedPointsMeanDistance(ref, out, icp);

      cout << "Paired points mean distance: " << meanDist << " m" << endl;*/
    }
  }
}
