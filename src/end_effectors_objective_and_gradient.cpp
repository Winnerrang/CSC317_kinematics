#include "end_effectors_objective_and_gradient.h"
#include "transformed_tips.h"
#include "kinematics_jacobian.h"
#include "copy_skeleton_at.h"
#include <iostream>

void end_effectors_objective_and_gradient(
  const Skeleton & skeleton,
  const Eigen::VectorXi & b,
  const Eigen::VectorXd & xb0,
  std::function<double(const Eigen::VectorXd &)> & f,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> & grad_f,
  std::function<void(Eigen::VectorXd &)> & proj_z)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code

  // need to make sure all the variable that got reference capture still exist
  // when these function are used
  f = [&](const Eigen::VectorXd & A)->double
  {
      auto new_skeleton = copy_skeleton_at(skeleton, A);
      auto tips_position = transformed_tips(new_skeleton, b);
      
      if (tips_position.size() != xb0.size()) {
          std::cout << "Size does not match between xb0 and tip_position" << std::endl;
          exit(1);
      }

      return 0.5 * (tips_position - xb0).squaredNorm();
  };
  grad_f = [&](const Eigen::VectorXd & A)->Eigen::VectorXd
  {
      Eigen::MatrixXd J;
      auto new_skeleton = copy_skeleton_at(skeleton, A);
      auto tip_position = transformed_tips(new_skeleton, b);
      kinematics_jacobian(new_skeleton, b, J);
      
      return J.transpose() * (tip_position - xb0);
  };
  proj_z = [&](Eigen::VectorXd & A)
  {
      assert(skeleton.size()*3 == A.size());
      for (int bone_idx = 0; bone_idx < skeleton.size(); bone_idx++) {
          auto bone = skeleton[bone_idx];
          auto xzx_max = bone.xzx_max;
          auto xzx_min = bone.xzx_min;
          for (int euler_angle = 0; euler_angle < 3; euler_angle++) {
              A(bone_idx * 3 + euler_angle) = std::max(xzx_min(euler_angle), 
                  std::min(xzx_max(euler_angle), A(bone_idx * 3 + euler_angle))
              );

          }
      }
  };
  /////////////////////////////////////////////////////////////////////////////
}
