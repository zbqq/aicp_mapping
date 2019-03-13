#ifndef AICP_CLASSIFICATION_COMMON_HPP_
#define AICP_CLASSIFICATION_COMMON_HPP_

#include <iostream>
#include <memory>
#include <vector>

#include <Eigen/Dense>

static void confusionMatrix(unsigned int tp, unsigned int tn, unsigned int fp, unsigned int fn) {
  std::cout << "====================================" << std::endl
            << "[Classifier Common] SVM Statistics: "   << std::endl
            << "====================================" << std::endl;

  std::cout << "TP: " << tp << ", "
            << "TN: " << tn << ", "
            << "FP: " << fp << ", "
            << "FN: " << fn << "\n";

  const double true_positive_rate = double(tp) / double(tp + fn);
  const double true_negative_rate = double(tn) / double(fp + tn);
  const double false_positive_rate = 1.0 - true_negative_rate;

  std::cout << "Accuracy (ACC): " << double(tp + tn) /
      double(tp + fp + tn + fn) << std::endl;
  std::cout << "Sensitivity (TPR): " << true_positive_rate << std::endl;
  std::cout << "Sensitivity (TPR): " << true_positive_rate << std::endl;
  std::cout << "(FPR): " << false_positive_rate << std::endl;
  std::cout << "Precision: " << double(tp) / double(tp + fp) << std::endl;
  std::cout << "Positive likelyhood ratio: " << true_positive_rate / false_positive_rate << std::endl;
  std::cout << "====================================" << std::endl;
}


struct ClassificationParams {
  std::string type;

  struct SVMParams {
    double threshold; // on prediction probability
    std::string trainingFile;
    std::string testingFile;
    std::string saveFile;
    std::string saveProbs;
    std::string modelLocation;
  } svm;

};

#endif
