#include "pointmatcher_registration.hpp"

namespace aicp{

  PointmatcherRegistration::PointmatcherRegistration()  {}

  PointmatcherRegistration::PointmatcherRegistration(const RegistrationParams& params) :
          params_(params) {
  }

  PointmatcherRegistration::~PointmatcherRegistration() {}

  //Templates
  void PointmatcherRegistration::registerClouds(pcl::PointCloud<pcl::PointXYZ>& cloud_ref, pcl::PointCloud<pcl::PointXYZ>& cloud_read, Eigen::Matrix4f &final_transform)
  {
    fromPCLToDataPoints(reference_cloud_, cloud_ref);
    fromPCLToDataPoints(reading_cloud_, cloud_read);

    registerClouds(final_transform);
  }

  void PointmatcherRegistration::registerClouds(pcl::PointCloud<pcl::PointXYZRGB>& cloud_ref, pcl::PointCloud<pcl::PointXYZRGB>& cloud_read, Eigen::Matrix4f &final_transform)
  {
    //fromPCLToDataPoints(reference_cloud_, cloud_ref);
    //fromPCLToDataPoints(reading_cloud_, cloud_read);

    //registerCsudo make installlouds(final_transform);
  }

  void PointmatcherRegistration::registerClouds(pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud_ref, pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud_read, Eigen::Matrix4f &final_transform)
  {
    //fromPCLToDataPoints(reference_cloud_, cloud_ref);
    //fromPCLToDataPoints(reading_cloud_, cloud_read);

    //registerClouds(final_transform);
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
  void PointmatcherRegistration::applyInitialization()
  {
    init_transform_ = parseTransformationDeg(params_.pointmatcher.initialTransform, 3);

    PM::Transformation* rigid_transform = PM::get().REG(Transformation).create("RigidTransformation");

    if (!rigid_transform->checkParameters(init_transform_)) {
      cerr << endl
        << "[Pointmatcher] Initial transformation is not rigid, identity will be used." << endl;
      init_transform_ = PM::TransformationParameters::Identity(4, 4);
    }
    else
      cout << "[Pointmatcher] Initialization: " << params_.pointmatcher.initialTransform << endl;

    // Manually transform reading to init position for visualization
    initialized_reading_ = rigid_transform->compute(reading_cloud_, init_transform_);
  }

  //Registration: Compute transform which aligns reading cloud onto the reference cloud.
  void PointmatcherRegistration::registerClouds(Eigen::Matrix4f &final_transform)
  {
    int cloud_dimension = reference_cloud_.getEuclideanDim();

    if (!(cloud_dimension == 3))
    {
      cerr << "[Pointmatcher] Invalid input point clouds dimension." << endl;
      exit(1);
    }

    //Params
    applyConfig();
    applyInitialization();

    // Compute the transformation
    PM::TransformationParameters T = icp_(reading_cloud_, reference_cloud_, init_transform_);
    //Ratio of how many points were used for error minimization (defined as TrimmedDistOutlierFilter ratio)
    cout << "[Pointmatcher] Accepted matches (inliers): " << (icp_.errorMinimizer->getWeightedPointUsedRatio())*100 << " %" << endl;
    // DEBUG pointmatcher functions:
    Eigen::MatrixXf covariance = (icp_.errorMinimizer->getCovariance());
    cout << "[Pointmatcher] Covariance: " << endl << covariance << endl;

    Eigen::EigenSolver<Eigen::MatrixXf> es(covariance);
    float sum_lambda = es.eigenvalues()(0,0).real()+es.eigenvalues()(1,0).real()+es.eigenvalues()(2,0).real()+
                       es.eigenvalues()(3,0).real()+es.eigenvalues()(4,0).real()+es.eigenvalues()(5,0).real();
    Eigen::VectorXf lambdas(6);
    lambdas << (es.eigenvalues()(0,0).real()/sum_lambda),
               (es.eigenvalues()(1,0).real()/sum_lambda),
               (es.eigenvalues()(2,0).real()/sum_lambda),
               (es.eigenvalues()(3,0).real()/sum_lambda),
               (es.eigenvalues()(4,0).real()/sum_lambda),
               (es.eigenvalues()(5,0).real()/sum_lambda);
    cout << "Covariance Eigenvalues:" << endl << es.eigenvalues() << endl;
    cout << "Lambdas:" << endl << lambdas << endl;

    // DEBUG pointmatcher added functions:
    Eigen::MatrixXf system_covariance = (icp_.errorMinimizer->getSystemCovariance());
    cout << "[Pointmatcher] System Covariance: " << endl << system_covariance << endl;

    Eigen::EigenSolver<Eigen::MatrixXf> es2(system_covariance);
    sum_lambda = es2.eigenvalues()(0,0).real()+es2.eigenvalues()(1,0).real()+es2.eigenvalues()(2,0).real()+
                       es2.eigenvalues()(3,0).real()+es2.eigenvalues()(4,0).real()+es2.eigenvalues()(5,0).real();
    Eigen::VectorXf system_lambdas(6);
    system_lambdas << (es2.eigenvalues()(0,0).real()/sum_lambda),
                      (es2.eigenvalues()(1,0).real()/sum_lambda),
                      (es2.eigenvalues()(2,0).real()/sum_lambda),
                      (es2.eigenvalues()(3,0).real()/sum_lambda),
                      (es2.eigenvalues()(4,0).real()/sum_lambda),
                      (es2.eigenvalues()(5,0).real()/sum_lambda);
    cout << "System Covariance Eigenvalues:" << endl << es2.eigenvalues() << endl;
    cout << "System Lambdas:" << endl << system_lambdas << endl;


    // Transform reading with T
    DP tmp_out_read_cloud_(reading_cloud_);
    icp_.transformations.apply(tmp_out_read_cloud_, T);

    out_read_cloud_ = tmp_out_read_cloud_;

    final_transform = T;

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
