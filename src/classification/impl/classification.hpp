#ifndef AICP_CLASSIFICATION_CREATE_CLASSIFIER_HPP_
#define AICP_CLASSIFICATION_CREATE_CLASSIFIER_HPP_

#include "aicpClassification/abstract_classification.hpp"
#include "aicpClassification/svm.hpp"

//#include <memory>

namespace aicp {

	static std::unique_ptr<AbstractClassification> create_classifier(const ClassificationParams& parameters) {
	  std::unique_ptr<AbstractClassification> classifier;
	  if (parameters.type == "SVM") {
	    classifier = std::unique_ptr<AbstractClassification>(new SVM(parameters));
	  } else {
	    std::cerr << "Invalid classification type " << parameters.type << "." << std::endl;
	  }
	  return classifier;
	}

}

#endif
