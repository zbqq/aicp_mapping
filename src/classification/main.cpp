// classification
#include "aicpClassification/classification.hpp"
#include "aicpClassification/common.hpp"

// yaml
#include "yaml-cpp/yaml.h" // read the yaml config

// project
#include "aicpCommonUtils/common.hpp" // CONFIG_LOC, PATH_SEPARATOR
#include "aicpCommonUtils/fileIO.h"

#include <memory> // unique_ptr

// csv to Eigen Matrix
#include <Eigen/Dense>
#include <vector>
#include <fstream>

// opencv
#include <opencv2/highgui/highgui.hpp>

using namespace Eigen;
using namespace aicp;

std::unique_ptr<AbstractClassification> classifier;

template<typename M>
M load_txt (const std::string & path) {
    std::ifstream indata;
    indata.open(path);
    std::string line;
    std::vector<double> values;
    uint rows = 0;
    while (std::getline(indata, line)) {
      std::stringstream lineStream(line);
      std::string cell;
      while (std::getline(lineStream, cell, ' ')) {
        values.push_back(std::stod(cell));
      }
      ++rows;
    }
    return Map<const Matrix<typename M::Scalar, M::RowsAtCompileTime, M::ColsAtCompileTime, RowMajor>>(values.data(), rows, values.size()/rows);
}

int main(int argc, char **argv) {

  ClassificationParams params;

  /*===================================
  =            YAML Config            =
  ===================================*/
  std::string yamlConfig_;
  YAML::Node yn_;

  yamlConfig_.append(CONFIG_LOC);
  yamlConfig_.append(PATH_SEPARATOR);
  yamlConfig_.append("aicp_config.yaml");

  yn_ = YAML::LoadFile(yamlConfig_);

  YAML::Node classificationNode = yn_["AICP"]["Classifier"];

  for (YAML::const_iterator it = classificationNode.begin(); it != classificationNode.end(); ++it) {
    const std::string key = it->first.as<std::string>();

    if (key.compare("type") == 0) {
      params.type = it->second.as<std::string>();
    }
  }

  if (params.type.compare("SVM") == 0) {

    YAML::Node svmNode = classificationNode["SVM"];

    for(YAML::const_iterator it=svmNode.begin();it != svmNode.end();++it) {
      const std::string key = it->first.as<std::string>();

      if(key.compare("threshold") == 0) {
        params.svm.threshold = it->second.as<double>();
      }
      else if(key.compare("trainingFile") == 0) {
        params.svm.trainingFile = it->second.as<std::string>();
      }
      else if(key.compare("testingFile") == 0) {
          params.svm.testingFile = it->second.as<std::string>();
      }
      else if(key.compare("saveFile") == 0) {
        params.svm.saveFile = it->second.as<std::string>();
      }
      else if(key.compare("saveProbs") == 0) {
        params.svm.saveProbs = it->second.as<std::string>();
      }
      else if(key.compare("modelLocation") == 0) {
        params.svm.modelLocation = it->second.as<std::string>();
      }
    }
  }

  std::cout << "============================" << std::endl
            << "Parsed YAML Config"           << std::endl
            << "============================" << std::endl;

  std::cout << "[Main] Classification Type: "       << params.type                    << std::endl;

  if(params.type.compare("SVM") == 0) {
    std::cout << "[Main] Acceptance Threshold: "    << params.svm.threshold           << std::endl;
    std::cout << "[Main] Training File: "           << params.svm.trainingFile        << std::endl;
    std::cout << "[Main] Testing File: "            << params.svm.testingFile         << std::endl;
    std::cout << "[Main] Saving Model To: "         << params.svm.saveFile            << std::endl;
    std::cout << "[Main] Saving Probs To: "         << params.svm.saveProbs           << std::endl;
    std::cout << "[Main] Loading Model From: "      << params.svm.modelLocation       << std::endl;
  }

  /*===================================
  =              Training             =
  ===================================*/

  MatrixXd training_data_all = load_txt<MatrixXd>(params.svm.trainingFile);
//  MatrixXd training_data_labels = load_txt<MatrixXd>(params.svm.trainingLabelsFile);
  int training_data_size = training_data_all.rows();

  classifier = create_classifier(params);

  MatrixXd training_data = training_data_all.block(0,1,training_data_size,2);
  training_data.col(1) = 100.0 * training_data.col(1);
  MatrixXd training_data_labels = training_data_all.block(0,3,training_data_size,1);
//  std::cout << "Training: " << std::endl;
//  std::cout << training_data << std::endl;

  classifier->train(training_data, training_data_labels);

//  /*===================================
//  =              Testing              =
//  ===================================*/

//  MatrixXd testing_data_all = load_txt<MatrixXd>(params.svm.testingFile);
////  MatrixXd testing_features_labels = load_txt<MatrixXd>(params.svm.testingLabelsFile);
//  int testing_data_size = testing_data_all.rows();

//  MatrixXd testing_data = testing_data_all.block(0,1,testing_data_size ,2);
//  testing_data.col(1) = 100.0 * testing_data.col(1);
//  MatrixXd testing_data_labels = testing_data_all.block(0,3,testing_data_size,1);
////  std::cout << "Testing: " << std::endl;
////  std::cout << testing_data << std::endl;

//  Eigen::MatrixXd probabilities;
//  classifier->test(testing_data, testing_data_labels, &probabilities);
////  std::cout << "Probabilities: " << std::endl;
////  std::cout << probabilities << std::endl;

//  for(int i = 0; i < probabilities.rows(); i++)
//  {
//    Eigen::MatrixXf probability = probabilities.row(i).cast <float> ();
//    writeLineToFile(probability, params.svm.saveProbs, i);
//  }

  /*===================================
  =     Testing and Visualization     =
  ===================================*/

    int width = 100, height = 100;
    cv::Mat image = cv::Mat::zeros(height, width, CV_8UC3);

    cv::Vec3b green(0,255,0), red(0,0,255);
    // Show the decision regions given by the SVM
    for (int i = 0; i < image.rows; ++i)
        for (int j = 0; j < image.cols; ++j)
        {
            MatrixXd testing_data(1,2);
            testing_data << j, i;
            MatrixXd testing_data_labels(1,1);
            testing_data_labels << 1.0;
            Eigen::MatrixXd probabilities;
            classifier->test(testing_data, testing_data_labels, &probabilities);

            std::cout << "[Example] Probability: " << probabilities(0,0) << std::endl;
            if (probabilities(0,0) <= params.svm.threshold)
                image.at<cv::Vec3b>(i,j) = green;
            else if (probabilities(0,0) > params.svm.threshold)
                image.at<cv::Vec3b>(i,j) = red;
//            float color = 255.0 * probabilities(0,0);
//            std::cout << "[Example] Color: " << color << std::endl;
//            cv::Vec3b scaled_color(255,color,color);
//            image.at<cv::Vec3b>(i,j) = scaled_color;
        }

    // Show the training data
    int thickness = -1;
    int lineType = 8;
    for (int i = 0; i < training_data_size; ++i)
    {
      if(training_data_labels(i,0) == 1)
        cv::circle(image, cv::Point((int)training_data(i,0),(int)training_data(i,1)), 0.1, cv::Scalar(0, 0, 0), thickness, lineType);
      else if(training_data_labels(i,0) == 0)
        cv::circle(image, cv::Point((int)training_data(i,0),(int)training_data(i,1)), 0.11, cv::Scalar(255, 255, 255), thickness, lineType);
    }

    cv::imwrite("result.png", image);        // save the image

    cv::imshow("SVM Simple Example", image); // show it to the user
    cv::waitKey(0);
}
