#ifndef AICP_OVERLAP_CREATE_REGISTRATOR_HPP_
#define AICP_OVERLAP_CREATE_REGISTRATOR_HPP_

#include "aicpOverlap/abstract_overlapper.hpp"
#include "aicpOverlap/octrees_overlap.hpp"

namespace aicp {

  static std::unique_ptr<AbstractOverlapper> create_overlapper(const OverlapParams& parameters) {
    std::unique_ptr<AbstractOverlapper> overlapper;
    if (parameters.type == "OctreeBased") {
      overlapper = std::unique_ptr<AbstractOverlapper>(new OctreesOverlap(parameters));
    }// else if (parameters.type == "") {
    //  registrator = std::unique_ptr<AbstractOverlapper>(new OtherOverlap(parameters));
    //} else {
    //  std::cerr << "Invalid registration type " << parameters.type << "." << std::endl;
    //}
    return overlapper;
  }
}

#endif
