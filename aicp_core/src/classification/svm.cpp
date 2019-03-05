#include "aicp_classification/svm.hpp"

namespace aicp {

  SVM::SVM(const ClassificationParams& params)
      : params_(params) {
    svm_ = cv::ml::SVM::create();
    // SVM's parameters
    svm_->setType(cv::ml::SVM::C_SVC);
    svm_->setKernel(cv::ml::SVM::POLY);
    svm_->setDegree(3);
    svm_->setTermCriteria(cv::TermCriteria(cv::TermCriteria::MAX_ITER, 100, 1e-6));
  }

  SVM::~SVM() {}

  void SVM::train(const Eigen::MatrixXd &training_data, const Eigen::MatrixXd &labels) {

    const unsigned int n_training_samples = training_data.rows();
    const unsigned int variables_dimension = training_data.cols();

    std::cout << "[Classifier] Training SVM with " << n_training_samples << " samples of dimension " << variables_dimension << "." << std::endl;

    // Set up training data
    cv::Mat1i opencv_labels(n_training_samples, 1, CV_32FC1);
    cv::Mat1f opencv_training_data(n_training_samples, variables_dimension, CV_32FC1);

    for (unsigned int i = 0u; i < n_training_samples; ++i) {
      for (unsigned int j = 0u; j < variables_dimension; ++j) {
        opencv_training_data.at<float>(i, j) = training_data(i, j);
      }
      opencv_labels.at<float>(i, 0) = labels(i, 0);
    }

    // Train the SVM
    cv::Ptr<cv::ml::TrainData> td = cv::ml::TrainData::create(opencv_training_data, cv::ml::ROW_SAMPLE, opencv_labels);
    // svm_->train(td);
    // or auto train
    svm_->trainAuto(td);

    if (params_.svm.saveFile.compare("") != 0) {
      save(params_.svm.saveFile.c_str());
    }
  }

  void SVM::test(const Eigen::MatrixXd &testing_data, Eigen::MatrixXd *probabilities) {
    int testing_data_size = testing_data.rows();
    Eigen::MatrixXd empty_labels = Eigen::MatrixXd::Zero(testing_data_size,1);

    test(testing_data, empty_labels, probabilities);
  }

  void SVM::test(const Eigen::MatrixXd &testing_data, const Eigen::MatrixXd &labels, Eigen::MatrixXd *probabilities) {

    // Load the classifier
    if (params_.svm.saveFile.compare("") != 0) {
      load(params_.svm.saveFile.c_str());
    }

    const unsigned int n_testing_samples = testing_data.rows();
    const unsigned int variables_dimension = testing_data.cols();

    std::cout << "[SVM] Testing SVM with " << n_testing_samples << " samples of dimension " << variables_dimension << "." << std::endl;

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

        int enable = 1;
        cv::Mat1f output;
        svm_->predict(sample, output, enable); // enable: enable probabilities
        double probability = 1.0 - 1.0 / (1.0 + exp(-output.at<float>(0, 0)));
        // std::cout << "[SVM] Output:" << output << std::endl;
        // std::cout << "[SVM] Probability:" << probability << std::endl;
        if (!labels.isZero() && probability >= params_.svm.threshold) { // high alignment risk
                                                    // --> expected failure (positive label = 1)
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
      if (!labels.isZero() && probabilities->rows() > 1)
        confusionMatrix(tp, tn, fp, fn);
    }
  }

  void SVM::save(const std::string &filename) {
    std::cout << "[SVM] Saving the classifier model to: " << filename << "." << std::endl;
    svm_->save(filename.c_str());
  }

  void SVM::load(const std::string &filename) {
    std::cout << "[SVM] Loading a classifier model from: " << filename << "." << std::endl;
    // in opencv3, cv::ml::SVM::load() creates a new instance, so you have to load your SVM like:
    svm_ = cv::ml::SVM::load(filename.c_str());
  }

}  // namespace aicp
