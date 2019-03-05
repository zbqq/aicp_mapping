#ifndef AICP_CLASSIFICATION_ABSTRACT_HPP_
#define AICP_CLASSIFICATION_ABSTRACT_HPP_

#include <opencv2/core/core.hpp>

#include "common.hpp"

namespace aicp {

	class AbstractClassification {
	public:
	  virtual void train(const Eigen::MatrixXd &training_data, const Eigen::MatrixXd &labels) = 0;
	  virtual void test(const Eigen::MatrixXd &testing_data, Eigen::MatrixXd *probabilities) = 0;
	  virtual void test(const Eigen::MatrixXd &testing_data, const Eigen::MatrixXd &labels, Eigen::MatrixXd *probabilities = NULL) = 0;
	  virtual void save(const std::string &filename) = 0;
	  virtual void load(const std::string &filename) = 0;
	};

}

#endif
