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

/**
 * Utility functions
 */

//convert formats in Eigen
Eigen::Isometry3d fromMatrix4fToIsometry3d(Eigen::Matrix4f matrix);

//compute angle between two vecotrs (returns degrees)
double angleBetweenVectors2d(const Eigen::Vector2d& v1,
                             const Eigen::Vector2d& v2);

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

void quat_to_euler(Eigen::Quaterniond q, double& roll, double& pitch, double& yaw);

Eigen::Quaterniond euler_to_quat(double roll, double pitch, double yaw);
