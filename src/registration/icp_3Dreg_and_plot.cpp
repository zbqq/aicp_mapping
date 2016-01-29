#include "icp_3Dreg_and_plot.hpp"
#include "vtkUtils.h"

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
  
  //================ Set up pronto visualizer ===============
  bool reset = 0;  
  pc_vis_ = new pronto_vis( lcm_->getUnderlyingLCM() );
  // obj: id name type reset
  // pts: id name type reset objcoll usergb rgb
  pc_vis_->obj_cfg_list.push_back( obj_cfg(60000, "Pose - Null", 5, reset) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(60001, "Cloud_Ref - Null", 1, reset, 60000, 1, {0.0, 0.1, 0.0}) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(60002, "Cloud_In - Null", 1, reset, 60000, 1, {0.0, 0.0, 1.0}) );  
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(60003, "Cloud_Result - Null", 1, reset, 60000, 1, {1.0, 0.0, 0.0}) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(60004, "Scan_Ref - Null", 1, reset, 60000, 1, {0.0, 0.1, 0.0}) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(60005, "Scan_In - Null", 1, reset, 60000, 1, {0.0, 0.0, 1.0}) );  
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(60006, "Scan_Result - Null", 1, reset, 60000, 1, {1.0, 0.0, 0.0}) );

  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(60007, "Cloud_Trans - Null", 1, reset, 60000, 1, {0.0, 1.0, 0.0}) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(60008, "Scan_Trans - Null", 1, reset, 60000, 1, {0.0, 1.0, 0.0}) );
  //==========================================================

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ref_ptr (new pcl::PointCloud<pcl::PointXYZRGB> ());
  reference_cloud_ = cloud_ref_ptr;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_ptr (new pcl::PointCloud<pcl::PointXYZRGB> ());
  transformed_input_cloud_ = cloud_in_ptr;
}

// Get transform after ICP alignment of reference and input cloud. Publish to LCM channel. 
void Registration::getICPTransform(DP &cloud_in, DP &cloud_ref)
{
  // Transform input clouds into a pcl PointXYZRGB and publish for visualization
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudB (new pcl::PointCloud<pcl::PointXYZRGB> ());
  fromDataPointsToPCL(cloud_ref, *reference_cloud_);
  fromDataPointsToPCL(cloud_in, *cloudB);

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
  PM::TransformationParameters initT = parseTransformationDeg(reg_cfg_.initTrans_, cloudDimension);

  PM::Transformation* rigidTrans;
  rigidTrans = PM::get().REG(Transformation).create("RigidTransformation");

  if (!rigidTrans->checkParameters(initT)) {
    cerr << endl
      << "Initial transformation is not rigid, identity will be used." << endl;
    initT = PM::TransformationParameters::Identity(cloudDimension+1,cloudDimension+1);
  }
  else
    cout << "Initialization: " << reg_cfg_.initTrans_ << endl;

  // Compute the transformation to express input in ref
  PM::TransformationParameters T = icp_(cloud_in, cloud_ref, initT);
  //Ratio of how many points were used for error minimization (defined as TrimmedDistOutlierFilter ratio)
  cout << "Matches used for minimization: " << (icp_.errorMinimizer->getWeightedPointUsedRatio())*100 << " %" << endl;

  // Transform input to express it in ref
  DP data_out(cloud_in);
  icp_.transformations.apply(data_out, T);

  out_cloud_ = data_out;

  // Plot input after initialization and after final transformation
  fromDataPointsToPCL(out_cloud_, *transformed_input_cloud_);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_trans (new pcl::PointCloud<pcl::PointXYZRGB> ());
  DP initializedInput = rigidTrans->compute(cloud_in, initT);
  fromDataPointsToPCL(initializedInput, *cloud_trans);

  // Publish clouds: plot in pronto visualizer
  publishCloud(60001, reference_cloud_);
  publishCloud(60002, cloudB);
  publishCloud(60003, transformed_input_cloud_);
  publishCloud(60007, cloud_trans);

  out_transform_ = T;
}

float Registration::hausdorffDistance(DP &ref, DP &out)
{
  // Test for retrieving Haussdorff distance (with outliers). We generate new matching module 
  // specifically for this purpose. 
  //
  // INPUTS:
  // ref: point cloud used as reference
  // out: aligned point cloud (using the transformation outputted by icp)
  
  // Structure to hold future match results
  PM::Matches matches;

  Parametrizable::Parameters params;
  params["knn"] =  toParam(1); // for Hausdorff distance, we only need the first closest point
  params["epsilon"] =  toParam(0);

  PM::Matcher* matcherHausdorff = PM::get().MatcherRegistrar.create("KDTreeMatcher", params);

  float quantile = 0.60;
  
  // max. distance from reading to reference
  matcherHausdorff->init(ref);
  matches = matcherHausdorff->findClosests(out);
  float maxDist1 = matches.getDistsQuantile(1.0);
  float maxDistRobust1 = matches.getDistsQuantile(quantile);

  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  // Store to VTK output cloud with distances from reference. 
  PM::Matrix values1(matches.dists.rows(), matches.dists.cols()); // (1 X nbPoints)
  for (int i = 0; i < matches.dists.cols(); i++)
  {
    if (matches.dists(0, i) != numeric_limits<float>::infinity())
    {
      values1(0, i) = sqrt(matches.dists(0, i));
    }
  }
  savePointCloudVTK("readHausdMatched.vtp", out, values1);

  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  // max. distance from reference to reading
  matcherHausdorff->init(out);
  matches = matcherHausdorff->findClosests(ref);
  float maxDist2 = matches.getDistsQuantile(1.0);
  float maxDistRobust2 = matches.getDistsQuantile(quantile);

  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  // Store to VTK reference cloud with distances from output. 
  PM::Matrix values2(matches.dists.rows(), matches.dists.cols()); // (1 X nbPoints)
  for (int i = 0; i < matches.dists.cols(); i++)
  {
    if (matches.dists(0, i) != numeric_limits<float>::infinity())
    {
      values2(0, i) = sqrt(matches.dists(0, i));
    }
  }
  savePointCloudVTK("refHausdMatched.vtp", ref, values2);

  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  float haussdorffDist = std::max(maxDist1, maxDist2);
  float haussdorffQuantileDist = std::max(maxDistRobust1, maxDistRobust2);

  //cout << "Haussdorff distance: " << std::sqrt(haussdorffDist) << " m" << endl;
  cout << "Haussdorff quantile distance (" << quantile << "): " << std::sqrt(haussdorffQuantileDist) <<  " m" << endl;  

  return std::sqrt(haussdorffDist);
}

float Registration::pairedPointsMeanDistance(DP &ref, DP &out, PM::ICP &icp)
{
  // Test for retrieving paired point mean distance without outliers. We reuse the same module used for 
  // the icp object.
  //
  // INPUTS:
  // ref: point cloud used as reference
  // data_out: aligned point cloud (using the transformation outputted by icp)
  // icp: icp object used to aligned the point clouds

  // Structure to hold future match results
  PM::Matches matches;
  
  // initiate the matching with unfiltered point cloud
  icp.matcher->init(ref);

  // extract closest points
  matches = icp.matcher->findClosests(out);

  /* An outlier filter removes or weights links between points in reading 
  and their matched points in reference, depending on some criteria.
  Criteria can be a fixed maximum authorized distance, a factor of the median distance, etc. 
  Points with zero weights are ignored in the subsequent minimization step. 
  So, once points have been matched and are linked, the outlier filter step attempts to remove links which do not correspond to true point correspondences. 
  The trimmed distance outlier filter does so by sorting links by their distance. 
  Points that are matched with a closer distance are less likely to be outliers. 
  The high distance matches in the upper 25% quantile are rejected (if ratio in config file set to 75%).
  */
  // weight paired points
  const PM::OutlierWeights outlierWeights = icp.outlierFilters.compute(out, ref, matches);
  
  // generate tuples of matched points and remove pairs with zero weight
  const PM::ErrorMinimizer::ErrorElements matchedPoints = icp.errorMinimizer->getMatchedPoints(out, ref, matches, outlierWeights);

  // extract relevant information for convenience
  const int dim = matchedPoints.reading.getEuclideanDim();
  const int nbMatchedPoints = matchedPoints.reading.getNbPoints(); 
  const PM::Matrix matchedRead = matchedPoints.reading.features.topRows(dim);
  const PM::Matrix matchedRef = matchedPoints.reference.features.topRows(dim);
  
  // compute mean distance
  const PM::Matrix dist = (matchedRead - matchedRef).colwise().norm(); // replace that by squaredNorm() to save computation time
  const float meanDist = dist.sum()/nbMatchedPoints;
  //cout << "Robust mean distance: " << meanDist << " m" << endl; 

  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  // Cloud which contains points belonging to the reference cloud. A matching point in the input cloud is associated to each of these. 
  DP matchedPointsRef = matchedPoints.reference;
  // Viceversa...
  DP matchedPointsRead = matchedPoints.reading;

  savePointCloudVTK("referenceMatched.vtp", matchedPointsRef, dist);
  savePointCloudVTK("readingMatched.vtp", matchedPointsRead);

  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  // Cloud which contains the points belonging to the input cloud which do not exist in the reference (non matching points).
  // store tuples of removed points (zero weight)
  PM::OutlierWeights notOutlierWeights(outlierWeights.rows(), outlierWeights.cols());
  for(int k = 0; k < outlierWeights.rows(); k++) // knn
  {
    for (int i = 0; i < outlierWeights.cols(); i++)
    {
      if (outlierWeights(k,i) != 0.0)
        notOutlierWeights(k,i) = 0;
      else
        notOutlierWeights(k,i) = 1;
    }
  }
  const PM::ErrorMinimizer::ErrorElements nonMatchedPoints = icp.errorMinimizer->getMatchedPoints(out, ref, matches, notOutlierWeights);
  DP nonMatchedPointsInput = nonMatchedPoints.reading;
  savePointCloudVTK("readingNonMatched.vtp", nonMatchedPointsInput);
  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

  return meanDist;
}

void Registration::publishCloud(int cloud_id, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{
  Isometry3dTime null_T = Isometry3dTime(1, Eigen::Isometry3d::Identity());
  pc_vis_->pose_to_lcm_from_list(60000, null_T);
  pc_vis_->ptcld_to_lcm_from_list(cloud_id, *cloud, 1, 1);
}



