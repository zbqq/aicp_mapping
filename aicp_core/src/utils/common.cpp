#include "aicp_utils/common.hpp"
#include <random>

Eigen::Isometry3d fromMatrix4fToIsometry3d(Eigen::Matrix4f matrix){
  Eigen::Isometry3d isometry;
  isometry.setIdentity();

  Eigen::Quaternionf quat(matrix.block<3,3>(0,0));
  Eigen::Quaterniond quatd;
  quatd = quat.cast <double> ();

  Eigen::Vector3f transl;
  transl = matrix.block<3,1>(0,3);
  Eigen::Vector3d transld;
  transld = transl.cast <double> ();

  isometry.translation() << transld;
  Eigen::Quaterniond quatE = Eigen::Quaterniond(quatd.w(), quatd.x(),
                                                quatd.y(), quatd.z());
  isometry.rotate(quatE);

  return isometry;
}

//extract integers from a string.
std::string extract_ints(std::ctype_base::mask category, std::string str, std::ctype<char> const& facet)
{
    using std::strlen;

    char const *begin = &str.front(),
               *end   = &str.back();

    auto res = facet.scan_is(category, begin, end);

    begin = &res[0];
    end   = &res[strlen(res)];

    return std::string(begin, end);
}

std::string extract_ints(std::string str)
{
    return extract_ints(std::ctype_base::digit, str,
         std::use_facet<std::ctype<char>>(std::locale("")));
}

//sample from Gaussian distribution
Eigen::VectorXf get_random_gaussian_variable(float mean, float std_deviation, int size)
{
  std::random_device rd;

  std::mt19937 e2(rd());

  std::normal_distribution<float> dist(mean, std_deviation);

  Eigen::VectorXf rand_variables(size);
  for (int n = 0; n < rand_variables.size(); n++) {
    rand_variables(n) = dist(e2);
  }

  return rand_variables;
}

// http://eigen.tuxfamily.org/bz/show_bug.cgi?id=1301
void quat_to_euler(Eigen::Quaterniond q, double& roll, double& pitch, double& yaw) {
  const double q0 = q.w();
  const double q1 = q.x();
  const double q2 = q.y();
  const double q3 = q.z();
  roll = atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2));
  pitch = asin(2*(q0*q2-q3*q1));
  yaw = atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3));
}
