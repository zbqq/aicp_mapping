#ifndef AICP_REGISTRATION_CREATE_REGISTRATOR_HPP_
#define AICP_REGISTRATION_CREATE_REGISTRATOR_HPP_

#include "aicpRegistration/abstract_registrator.hpp"
#include "aicpRegistration/pointmatcher_registration.hpp"

namespace aicp {

  static std::unique_ptr<AbstractRegistrator> create_registrator(const RegistrationParams& parameters) {
    std::unique_ptr<AbstractRegistrator> registrator;
    if (parameters.type == "Pointmatcher") {
      registrator = std::unique_ptr<AbstractRegistrator>(new PointmatcherRegistration(parameters));
    } else if (parameters.type == "GICP") {
    //  registrator = std::unique_ptr<AbstractRegistrator>(new GICPRegistration(parameters));
    } else {
      std::cerr << "Invalid registration type " << parameters.type << "." << std::endl;
    }
    return registrator;
  }
}

#endif
