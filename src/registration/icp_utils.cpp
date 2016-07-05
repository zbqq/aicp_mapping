#include "icp_utils.h"

/* Compute some metrics given registration between a reference and an input cloud 
(Haussdorff distance, Haussdorff quantile distance and Robust mean distance in meters). */

float hausdorffDistance(DP &ref, DP &out)
{
  const char *filename = "readHausdMatched.vtk";
  return hausdorffDistance(ref, out, filename);
}

float hausdorffDistance(DP &ref, DP &out, const char *filename)
{
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

  //savePointCloudVTP("referenceMatched.vtp", matchedPointsRef, dist);
  //savePointCloudVTP("readingMatched.vtp", matchedPointsRead);

  savePointCloudVTK(filename, matchedPointsRef, dist);
  savePointCloudVTK("readingMatched.vtk", matchedPointsRead, dist);

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

/* Get the line whose index is given as argument from file. */

string readLineFromFile(string& filename, int line_number)
{
  string lines, line;
  int index = 0;

  ifstream trasfFile (filename);
  if (trasfFile.is_open())
  {
    while ( getline (trasfFile, lines) )
    {
      if (index == line_number)
      {
      	getline (trasfFile, line);
      }
      index ++;
    }
    trasfFile.close();
  }
  else cout << "Unable to open init tranform file."; 

  return line;
}

/* Get a transformation matrix (of the type defined in the libpointmatcher library)
given a string containing info about a translation on the plane x-y and a rotation 
about the vertical axis z, i.e. [x,y,theta] (meters,meters,radians). */

PM::TransformationParameters parseTransformation(string& transform, const int cloudDimension) 
{
  PM::TransformationParameters parsedTrans;
  parsedTrans = PM::TransformationParameters::Identity(
        cloudDimension+1,cloudDimension+1);

  transform.erase(std::remove(transform.begin(), transform.end(), '['),
            transform.end());
  transform.erase(std::remove(transform.begin(), transform.end(), ']'),
            transform.end());
  std::replace( transform.begin(), transform.end(), ',', ' ');
  std::replace( transform.begin(), transform.end(), ';', ' ');

  float transValues[3] = {0};
  stringstream transStringStream(transform);
  for( int i = 0; i < 3; i++) {
    if(!(transStringStream >> transValues[i])) {
      cerr << "An error occured while trying to parse the initial "
         << "transformation." << endl
         << "No initial transformation will be used" << endl;
      return parsedTrans;
    }
  }

  for( int i = 0; i < 3; i++) {
    if (i == 2)
    {
      parsedTrans(i-2,i-2) = cos ( transValues[i] );
      parsedTrans(i-2,i-1) = - sin ( transValues[i] );
      parsedTrans(i-1,i-2) = sin ( transValues[i] );
      parsedTrans(i-1,i-1) = cos ( transValues[i] );
    }
    else
    {
      parsedTrans(i,cloudDimension) = transValues[i];
    }
  }

  //cout << "Parsed initial transformation:" << endl << parsedTrans << endl;

  return parsedTrans;
}

/* Get a transformation matrix (of the type defined in the libpointmatcher library)
given a string containing info about a translation on the plane x-y and a rotation 
about the vertical axis z, i.e. [x,y,theta] (meters,meters,degrees). */

PM::TransformationParameters parseTransformationDeg(string& transform, const int cloudDimension) 
{
  PM::TransformationParameters parsedTrans;
  parsedTrans = PM::TransformationParameters::Identity(
        cloudDimension+1,cloudDimension+1);

  transform.erase(std::remove(transform.begin(), transform.end(), '['),
            transform.end());
  transform.erase(std::remove(transform.begin(), transform.end(), ']'),
            transform.end());
  std::replace( transform.begin(), transform.end(), ',', ' ');
  std::replace( transform.begin(), transform.end(), ';', ' ');

  float transValues[3] = {0};
  stringstream transStringStream(transform);
  for( int i = 0; i < 3; i++) {
    if(!(transStringStream >> transValues[i])) {
      cerr << "An error occured while trying to parse the initial "
         << "transformation." << endl
         << "No initial transformation will be used" << endl;
      return parsedTrans;
    }
  }

  for( int i = 0; i < 3; i++) {
    if (i == 2)
    {
      parsedTrans(i-2,i-2) = cos ( transValues[i] * M_PI / 180.0 );
      parsedTrans(i-2,i-1) = - sin ( transValues[i] * M_PI / 180.0 );
      parsedTrans(i-1,i-2) = sin ( transValues[i] * M_PI / 180.0 );
      parsedTrans(i-1,i-1) = cos ( transValues[i] * M_PI / 180.0 );
    }
    else
    {
      parsedTrans(i,cloudDimension) = transValues[i];
    }
  }

  //cout << "Parsed initial transformation:" << endl << parsedTrans << endl;

  return parsedTrans;
}

void fromDataPointsToPCL(DP &cloud_in, pcl::PointCloud<pcl::PointXYZRGB> &cloud_out)
{
  cloud_out.points.resize(cloud_in.getNbPoints());
  for (int i = 0; i < cloud_in.getNbPoints(); i++) {
    cloud_out.points[i].x = (cloud_in.features.col(i))[0];
    cloud_out.points[i].y = (cloud_in.features.col(i))[1];
    cloud_out.points[i].z = (cloud_in.features.col(i))[2];
    //cout << "i=" << i << " " << cloud_out.points[i].x << " " << cloud_out.points[i].y << " " << cloud_out.points[i].z << endl;
  }  
  cloud_out.width = cloud_out.points.size();
  cloud_out.height = 1;
}

void fromPCLToDataPoints(DP &cloud_out, pcl::PointCloud<pcl::PointXYZRGB> &cloud_in)
{
  // parse points
  int pointCount = cloud_in.width;
 
  PM::Matrix features(4, pointCount);

  for (int p = 0; p < pointCount; ++p)
  {
    features(0, p) = cloud_in.points[p].x;
    features(1, p) = cloud_in.points[p].y;
    features(2, p) = cloud_in.points[p].z;
    features(3, p) = 1.0;
  }
  cloud_out.addFeature("x", features.row(0));
  cloud_out.addFeature("y", features.row(1));
  cloud_out.addFeature("z", features.row(2));
  cloud_out.addFeature("pad", features.row(3));

  // NOTE: We use PointXYZRGB as a point structure representing Euclidean xyz only. The RGB field of the structure 
  // is not set because just one color should represent the whole cloud 
  // (see http://docs.pointclouds.org/1.7.0/point__types_8hpp_source.html, line 871). 
  // Instead we have a different color for each point in the cloud. 
}

void writeTransformToFile(Eigen::MatrixXf &transformations, string out_file, int num_clouds)
{
  std::vector<string> v;
  for (int i = 0; i < num_clouds-1; i++)
  {
    for (int j = 1+i; j < num_clouds; j++)
    {
      string str;
      str.append(to_string(i));
      str.append(to_string(j));
      v.push_back(str);
    }
  }

  ofstream file (out_file);

  if (file.is_open())
  {
    //file << "# x y theta\n";
    for (int i = 0; i < v.size(); i++)
    {
      file << v[i] << " " << transformations(0,i) << " " << transformations(1,i) 
      << " " << transformations(2,i) << endl;
    }
    file.close();
  }
  else cout << "Unable to open file";
  cout << "Written file: " << out_file << endl;
}

void writeLineToFile(Eigen::MatrixXf &values, string out_file, int line_number)
{
  // given values (row vector) store it as a line in a file with line number as first value
  ofstream file (out_file, std::ios_base::app | std::ios_base::out);

  if (file.is_open())
  {
    //file << "# 0 1 ... \n";
    file << line_number << " ";
    for (int i = 0; i < values.cols(); i++)
    {
      file << values(0,i) << " ";
    }
    file << endl;
    file.close();
  }
  else cout << "Unable to open file";
  cout << "Written file: " << out_file << endl;
}

void replaceRatioConfigFile(string in_file, string out_file, float ratio)
{
  std::ifstream in;
  in.open(in_file, std::fstream::in);
  std::ofstream out;
  out.open(out_file, std::ofstream::out);
  if (!in)
  {
    cerr << "Could not open config file for params update." << "\n";
  }
  if (!out)
  {
    cerr << "Could not open config file for params update." << "\n";
  }

  string word_to_replace = "ratio: ";
  std::stringstream word_to_replace_with_tmp;
  word_to_replace_with_tmp << "ratio: ";
  word_to_replace_with_tmp << ratio;
  string word_to_replace_with = word_to_replace_with_tmp.str();

  string line;
  size_t len = word_to_replace.length() + 4;
  while (!in.eof())
  {
    getline(in, line);
    size_t pos = line.find(word_to_replace);
    if (pos != string::npos)
    {
      line.replace(pos, len, word_to_replace_with);
    }
    out << line << '\n';
  }
  in.close();
  out.close();
}