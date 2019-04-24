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

double angleBetweenVectors2d(const Eigen::Vector2d &v1,
                             const Eigen::Vector2d &v2){
    return atan2(v1(0)*v2(1) - v1(1)*v2(0), v1(0)*v2(0) + v1(1)*v2(1)) * 180.0 / M_PI;
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


Eigen::Quaterniond euler_to_quat(double roll, double pitch, double yaw) {
  
  // This conversion function introduces a NaN in Eigen Rotations when:
  // roll == pi , pitch,yaw =0    ... or other combinations.
  // cos(pi) ~=0 but not exactly 0 
  if ( ((roll==M_PI) && (pitch ==0)) && (yaw ==0)){
    return  Eigen::Quaterniond(0,1,0,0);
  }else if( ((pitch==M_PI) && (roll ==0)) && (yaw ==0)){
    return  Eigen::Quaterniond(0,0,1,0);
  }else if( ((yaw==M_PI) && (roll ==0)) && (pitch ==0)){
    return  Eigen::Quaterniond(0,0,0,1);
  }
  
  double sy = sin(yaw*0.5);
  double cy = cos(yaw*0.5);
  double sp = sin(pitch*0.5);
  double cp = cos(pitch*0.5);
  double sr = sin(roll*0.5);
  double cr = cos(roll*0.5);
  double w = cr*cp*cy + sr*sp*sy;
  double x = sr*cp*cy - cr*sp*sy;
  double y = cr*sp*cy + sr*cp*sy;
  double z = cr*cp*sy - sr*sp*cy;
  return Eigen::Quaterniond(w,x,y,z);
}
