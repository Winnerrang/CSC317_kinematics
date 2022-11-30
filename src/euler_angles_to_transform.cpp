#include "euler_angles_to_transform.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <iostream>

Eigen::Affine3d euler_angles_to_transform(
  const Eigen::Vector3d & xzx)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code
  Eigen::Affine3d A;
  Eigen::Matrix3d R;

  float degToRad = M_PI / 180;


  R = Eigen::AngleAxisd(xzx[2] * degToRad, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(xzx[1] * degToRad, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(xzx[0] * degToRad, Eigen::Vector3d::UnitX());
  
;
  A.matrix() << R, Eigen::Vector3d::Zero(),
                0, 0, 0, 1;
  std::cout << A.matrix() << std::endl;
  return A;
  /////////////////////////////////////////////////////////////////////////////
}
