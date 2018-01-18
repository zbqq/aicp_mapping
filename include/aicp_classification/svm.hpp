#ifndef AICP_CLASSIFICATION_IMPL_RF_HPP_
#define AICP_CLASSIFICATION_IMPL_RF_HPP_

// classification
#include "aicp_classification/common.hpp"
#include "aicp_classification/abstract_classification.hpp"

// opencv
#include <opencv2/core/core.hpp>
#include <opencv2/ml/ml.hpp>

// nabo
//#include <nabo/nabo.h>

// eigen
#include <Eigen/Dense>

namespace aicp {

  class SVM : public AbstractClassification {
  public:
    SVM();
    explicit SVM(const ClassificationParams& params);
    ~SVM();

    virtual void train(const Eigen::MatrixXd &training_data, const Eigen::MatrixXd &labels);
    virtual void test(const Eigen::MatrixXd &testing_data, Eigen::MatrixXd *probabilities);
    virtual void test(const Eigen::MatrixXd &testing_data, const Eigen::MatrixXd &labels, Eigen::MatrixXd *probabilities = NULL);
    virtual void save(const std::string &filename);
    virtual void load(const std::string &filename);

  private:
    ClassificationParams params_;
    cv::ml::SVM* svm_;
    //CvSVM svm_;

//    // all 3 are indexed in the same way
//    std::vector<unsigned int> target_cloud_segments_ids_;
//    std::vector<pcl::PointXYZ> target_cloud_centroids_;
//    std::vector<Eigen::MatrixXf> target_cloud_features_;
//    static constexpr unsigned int kMinNumberSegmentInTargetCloud = 50u; // minimum number in target map in order to perform matching

//    Eigen::MatrixXf target_matrix_;
  };

}

#endif
