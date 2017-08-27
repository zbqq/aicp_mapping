#include "svm.hpp"

namespace aicp {

  SVM::SVM(const ClassificationParams& params)
      : params_(params) {
  }

  SVM::~SVM() {}

  void SVM::train(const Eigen::MatrixXd &training_data, const Eigen::MatrixXd &labels) {

    const unsigned int n_training_samples = training_data.rows();
    const unsigned int variables_dimension = training_data.cols();

    std::cout << "[Classifier] Training SVM with " << n_training_samples << " samples of dimension " << variables_dimension << "." << std::endl;

    // Set up training data
    cv::Mat opencv_labels(n_training_samples, 1, CV_32FC1);
    cv::Mat opencv_training_data(n_training_samples, variables_dimension, CV_32FC1);

    for (unsigned int i = 0u; i < n_training_samples; ++i) {
      for (unsigned int j = 0u; j < variables_dimension; ++j) {
        opencv_training_data.at<float>(i, j) = training_data(i, j);
      }
      opencv_labels.at<float>(i, 0) = labels(i, 0);
    }

    // SVM's parameters
    CvSVMParams svm_params;
    svm_params.svm_type = CvSVM::C_SVC;
    svm_params.kernel_type = CvSVM::POLY;
    svm_params.degree = 3;
    svm_params.term_crit = cvTermCriteria(CV_TERMCRIT_ITER, 100, 1e-6);

    // Train the SVM
    svm_.train(opencv_training_data, opencv_labels, cv::Mat(), cv::Mat(), svm_params);

    if (params_.svm.saveFile.compare("") != 0) {
      std::cout << "[Classifier] Saving the classifier to: " << params_.svm.saveFile << "." << std::endl;
      svm_.save(params_.svm.saveFile.c_str());
    }
  }


  void SVM::test(const Eigen::MatrixXd &testing_data, const Eigen::MatrixXd &labels, Eigen::MatrixXd *probabilities) {

    // Load the classifier
    if (params_.svm.saveFile.compare("") != 0) {
      std::cout << "[Classifier] Loading the classifier from: " << params_.svm.saveFile << "." << std::endl;
      svm_.load(params_.svm.saveFile.c_str());
    }

    const unsigned int n_testing_samples = testing_data.rows();
    const unsigned int variables_dimension = testing_data.cols();

    std::cout << "[Classifier] Testing SVM with " << n_testing_samples << " samples of dimension " << variables_dimension << "." << std::endl;

    if (probabilities != NULL)
      probabilities->resize(n_testing_samples, 1);

    if (n_testing_samples > 0u) {
      unsigned int tp = 0u, fp = 0u, tn = 0u, fn = 0u;

      for (size_t i = 0u; i < n_testing_samples; ++i) {

        // retrieve one sample
        cv::Mat sample(1, variables_dimension, CV_32FC1);
        for (size_t j = 0u; j < variables_dimension; j++) {
          sample.at<float>(j) = testing_data(i, j);
        }

        double decision = svm_.predict(sample, true); // true to enable probabilities
        double probability = 1.0 / (1.0 + exp(-decision));
//        double probability = svm_.predict(sample);
        if (probability <= params_.svm.threshold) { // high alignment risk
                                                    // --> expected failure
          if (labels(i, 0) == 1.0) {
            ++tp;
          } else {
            ++fp;
          }
        } else {
          if (labels(i, 0) == 0.0) {
            ++tn;
          } else {
            ++fn;
          }
        }

        if (probabilities != NULL) {
          (*probabilities)(i, 0) = probability;
        }
      }
      confusionMatrix(tp, tn, fp, fn);
    }
  }

  void SVM::save(const std::string &filename) {
    std::cout << "[Classifier] Saving the classifier model to: " << filename << "." << std::endl;
    svm_.save(filename.c_str());
  }

  void SVM::load(const std::string &filename) {
    std::cout << "[Classifier] Loading a classifier model from: " << filename << "." << std::endl;
    svm_.load(filename.c_str());
  }

}  // namespace aicp
