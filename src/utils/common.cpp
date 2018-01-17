#include "common.hpp"

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
