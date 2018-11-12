#pragma once

#include <string> // std::string
#include <math.h> // M_PI
#include <stack> // tictoc
#include <iostream> // cout

//Eigen
#include <Eigen/Dense>

// windows or unix user, get the separator correctly
#if defined(WIN32) || defined(_WIN32)
  const std::string PATH_SEPARATOR="\\";
#else
  const std::string PATH_SEPARATOR="/";
#endif

// if for some reason this is not defined in the cmake
// make the default project location point to RPG_BASE/software/aicp
#ifdef PROJECTDIR
  #define PROJECT_LOC PROJECTDIR
#else
  #define RPG_BASE getenv("RPG_BASE")
  const std::string PROJECT_LOC = std::string(RPG_BASE) + PATH_SEPARATOR + std::string("software") + PATH_SEPARATOR + std::string("aicp");
#endif

#ifdef PROJECT_LOC
  const std::string CONFIG_LOC = PROJECT_LOC + PATH_SEPARATOR + std::string("config");
#else
  #ifndef RPG_BASE
    #define RPG_BASE getenv("RPG_BASE")
  #endif
  const std::string CONFIG_LOC = std::string(RPG_BASE) + PATH_SEPARATOR + std::string("software") + PATH_SEPARATOR + std::string("aicp") + PATH_SEPARATOR + std::string("config");
#endif

#ifdef PROJECT_LOC
  const std::string FILTERS_CONFIG_LOC = PROJECT_LOC + PATH_SEPARATOR + std::string("filters_config");
#else
  #ifndef RPG_BASE
    #define RPG_BASE getenv("RPG_BASE")
  #endif
  const std::string FILTERS_CONFIG_LOC = std::string(RPG_BASE) + PATH_SEPARATOR + std::string("software") + PATH_SEPARATOR + std::string("aicp") + PATH_SEPARATOR + std::string("filters_config");
#endif

#ifdef PROJECT_LOC
  const std::string DATA_LOC = PROJECT_LOC + PATH_SEPARATOR + std::string("data");
#else
  #ifndef RPG_BASE
    #define RPG_BASE getenv("RPG_BASE")
  #endif
  const std::string DATA_LOC = std::string(RPG_BASE) + PATH_SEPARATOR + std::string("software") + PATH_SEPARATOR + std::string("aicp") + PATH_SEPARATOR + std::string("data");
#endif

/**
 * Utility functions
 */

//convert formats in Eigen
Eigen::Isometry3d fromMatrix4fToIsometry3d(Eigen::Matrix4f matrix);

//swapping two values.
template<typename T>
inline bool swap_if_gt(T& a, T& b) {
  if (a > b) {
    std::swap(a, b);
    return true;
  }
  return false;
}

inline double toRad(double deg) { return (deg * M_PI / 180); }
inline double toDeg(double rad) { return (rad * 180 / M_PI); }

//extract integers from a string.
std::string extract_ints(std::ctype_base::mask category, std::string str, std::ctype<char> const& facet);

std::string extract_ints(std::string str);

//sample from Gaussian distribution
Eigen::VectorXf get_random_gaussian_variable(float mean, float std_deviation, int size);


