#include "aicp_utils/icpMonitor.h"

/* Compute metrics (implemented in libpointmatcher) given registration between a reference and an input cloud
(Haussdorff distance, Haussdorff quantile distance and Robust mean distance in meters). */

float hausdorffDistance(DP &ref, DP &out)
{
  const char *filename = "readHausdMatched.vtk";
  return hausdorffDistance(ref, out, filename);
}

float hausdorffDistance(DP &ref, DP &out, const char *filename)
{
  using namespace PointMatcherSupport;
  // Compute Haussdorff distance. Save to file (.vtp and .vtk extensions) entire point clouds (with outliers). 
  // A custom field (distance between matches) is associated to each cloud.
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
  //savePointCloudVTP("readHausdMatched.vtp", out, values1);
  //savePointCloudVTK(filename, out, values1);

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
  //savePointCloudVTP("refHausdMatched.vtp", ref, values2);

  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  float haussdorffDist = std::max(maxDist1, maxDist2);
  float haussdorffQuantileDist = std::max(maxDistRobust1, maxDistRobust2);

  //cout << "Haussdorff distance: " << std::sqrt(haussdorffDist) << " m" << endl;
  //cout << "Haussdorff quantile distance (" << quantile << "): " << std::sqrt(haussdorffQuantileDist) <<  " m" << endl;  

  return std::sqrt(haussdorffDist);
}

PM::Matrix distancesKNN(DP &A, DP &B)
{
  const char *filename = "readDistances.vtk";
  return distancesKNN(A, B, filename);
}

PM::Matrix distancesKNN(DP &A, DP &B, const char *filename)
{
  using namespace PointMatcherSupport;
  // Compute distance nearest neighbors between 2 whole clouds. Save to file (.vtp and .vtk extensions) entire point clouds.
  // A custom field (distance between matches) is associated to each cloud.
  //
  // INPUTS:
  // A: point cloud used as reference
  // B: stored cloud with distance of each of its points from NN in cloud A

  PM::Matches matches;

  Parametrizable::Parameters params;
  params["knn"] =  toParam(1); // for Hausdorff distance, we only need the first closest point
  params["epsilon"] =  toParam(0);

  PM::Matcher* matcher = PM::get().MatcherRegistrar.create("KDTreeMatcher", params);

  // from reading to reference
  matcher->init(A);
  matches = matcher->findClosests(B);

  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  // Store to VTK output cloud with distances from reference.
  PM::Matrix values(matches.dists.rows(), matches.dists.cols()); // (1 X nbPoints)
  for (int i = 0; i < matches.dists.cols(); i++)
  {
    if (matches.dists(0, i) != numeric_limits<float>::infinity())
    {
      values(0, i) = sqrt(matches.dists(0, i));
    }
  }
  //savePointCloudVTP("readDistances.vtp", B, values);
  savePointCloudVTK(filename, B, values);

  int nbValidMatches = 0;
  float dist = 0;
  for (int i = 0; i < values.cols(); i++ )
  {
    if(values(0, i) <= 0.20) // distances bigger than maxAcceptedDist are not valid
    {
      dist = dist + values(0, i);
      nbValidMatches ++;
    }
  }
  const float meanDist = dist/nbValidMatches;
  cout << "Mean distance B from KNN in A (maxAcceptedDist = 0.20m): " << meanDist << " m" << endl;

  return values;
}

float pairedPointsMeanDistance(DP &ref, DP &out, PM::ICP &icp)
{
  const char *filename = "referenceMatched.vtk";
  return pairedPointsMeanDistance(ref, out, icp, filename);  
}

float pairedPointsMeanDistance(DP &ref, DP &out, PM::ICP &icp, const char *filename)
{
  // Compute paired point mean distance. Save to file (.vtp and .vtk extensions) point clouds made of matches (without outliers). 
  // A custom field (distance between matches) is associated to each cloud.
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

  // An outlier filter removes or weights links between points in reading 
  // and their matched points in reference, depending on some criteria.
  // Criteria can be a fixed maximum authorized distance, 
  // a factor of the median distance, etc. (http://docs.ros.org/hydro/api/libpointmatcher/html/OutlierFiltersImpl_8cpp_source.html)
  // Points with zero weights are ignored in the subsequent minimization step. 
  // So, once points have been matched and are linked, the outlier filter step attempts to remove links which do not correspond to true point correspondences. 
  // The trimmed distance outlier filter does so by sorting links by their distance. 
  // Points that are matched with a closer distance are less likely to be outliers. 
  // The high distance matches in the upper 25% quantile are rejected (if ratio in config file set to 75%).
  //
  // weight paired points
  const PM::OutlierWeights outlierWeights = icp.outlierFilters.compute(out, ref, matches);
  
  // generate tuples of matched points and remove pairs with zero weight
  // simalpha: this is disabled after catkinize
  // (error: ‘struct PointMatcher<float>::ErrorMinimizer’ has no member named ‘getMatchedPoints’)
  std::cout << "[ICP Monitor] pairedPointsMeanDistance(...): DISABLED!\n";
  const PM::ErrorMinimizer::ErrorElements matchedPoints;// = icp.errorMinimizer->getMatchedPoints(out, ref, matches, outlierWeights);

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
  /*
  // Cloud which contains points belonging to the reference cloud. A matching point in the input cloud is associated to each of these.
  DP matchedPointsRef = matchedPoints.reference;
  // Viceversa...
  DP matchedPointsRead = matchedPoints.reading;

  //savePointCloudVTP("referenceMatched.vtp", matchedPointsRef, dist);
  //savePointCloudVTP("readingMatched.vtp", matchedPointsRead);

  savePointCloudVTK(filename, matchedPointsRef, dist);
  savePointCloudVTK("readingMatched.vtk", matchedPointsRead, dist);
  */
  /*
  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  // Complement:
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
  //savePointCloudVTP("readingNonMatched.vtp", nonMatchedPointsInput);
  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
  */

  return meanDist;
}

void getResidualError(PM::ICP &icp, float overlap, float &meanDist, float &medDist, float &quantDist)
{
  const PM::ErrorMinimizer::ErrorElements matchedPoints = icp.errorMinimizer->getErrorElements();
  // extract relevant information for convenience
  const int dim = matchedPoints.reading.getEuclideanDim();
  const int nbMatchedPoints = matchedPoints.reading.getNbPoints();
  const PM::Matrix matchedRead = matchedPoints.reading.features.topRows(dim);
  const PM::Matrix matchedRef = matchedPoints.reference.features.topRows(dim);

  // compute mean and quantile residual distance between accepted matches
  const PM::Matrix dist = (matchedRead - matchedRef).colwise().squaredNorm(); // replace that by squaredNorm() to save computation time

  meanDist = dist.sum()/nbMatchedPoints;
  medDist = matchedPoints.matches.getDistsQuantile(0.50);
  quantDist = matchedPoints.matches.getDistsQuantile(overlap);

  //cout << "Mean Residual Distance: " << meanDist << " m" << endl;
  //cout << "Median Residual Distance: " << medDist << " m" << endl;
  //cout << "Quantile " << overlap << " Residual Distance: " <<  quantDist << " m" << endl;
}

Eigen::Isometry3d getTransfParamAsIsometry3d(PM::TransformationParameters T){
  Eigen::Isometry3d pose_iso;
  pose_iso.setIdentity();

  Eigen::Matrix3f rot;
  rot << T.block(0,0,3,3);
  Eigen::Quaternionf quat(rot);
  Eigen::Quaterniond quatd;
  quatd = quat.cast <double> ();

  Eigen::Vector3f transl;
  transl << T(0,3), T(1,3), T(2,3);
  Eigen::Vector3d transld;
  transld = transl.cast <double> ();

  pose_iso.translation() << transld;
  Eigen::Quaterniond quatE = Eigen::Quaterniond(quatd.w(), quatd.x(),
                                                quatd.y(), quatd.z());
  pose_iso.rotate(quatE);

  return pose_iso;
}
