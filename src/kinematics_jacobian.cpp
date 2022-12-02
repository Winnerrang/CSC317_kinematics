#include "kinematics_jacobian.h"
#include "transformed_tips.h"
#include <iostream>
#include "copy_skeleton_at.h"

void kinematics_jacobian(
  const Skeleton & skeleton,
  const Eigen::VectorXi & b,
  Eigen::MatrixXd & J)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code
	J = Eigen::MatrixXd::Zero(b.size() * 3, skeleton.size() * 3);
	double h = 1e-7;
	Eigen::VectorXd orig_theta(skeleton.size() * 3);

	for (int bone_index = 0; bone_index < skeleton.size(); bone_index++) {
		auto bone = skeleton[bone_index];
		auto rot = skeleton[bone_index].xzx;
		orig_theta(bone_index * 3) = rot.x();
		orig_theta(bone_index * 3 + 1) = rot.y();
		orig_theta(bone_index * 3 + 2) = rot.z();
	}

	for (int bone_index = 0; bone_index < skeleton.size(); bone_index++) {
		for (int euler_angle = 0; euler_angle < 3; euler_angle++) {
			Eigen::VectorXd delta = Eigen::VectorXd::Zero(3 * skeleton.size());
			delta(3 * bone_index + euler_angle) = 1;
			auto delta0 = orig_theta - h * delta;
			auto skeleton0 = copy_skeleton_at(skeleton, delta0);
			auto position0 = transformed_tips(skeleton0, b);

			auto delta1 = orig_theta + h * delta;
			auto skeleton1 = copy_skeleton_at(skeleton, delta1);
			auto position1 = transformed_tips(skeleton1, b);

			J.col(3 * bone_index + euler_angle) = (position1 - position0) / (2 * h);
		}

	}
  
  /////////////////////////////////////////////////////////////////////////////
}
