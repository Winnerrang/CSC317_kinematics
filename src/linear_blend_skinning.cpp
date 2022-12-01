#include "linear_blend_skinning.h"

void linear_blend_skinning(
  const Eigen::MatrixXd & V,
  const Skeleton & skeleton,
  const std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > & T,
  const Eigen::MatrixXd & W,
  Eigen::MatrixXd & U)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code
	U.resize(V.rows(), V.cols());

	for (int v_index = 0; v_index < V.rows(); v_index++) {
		auto v = V.row(v_index);

		Eigen::Vector3d new_v = Eigen::Vector3d::Zero();
		
		for (int bone_index = 0; bone_index < skeleton.size(); bone_index++) {
			auto bone = skeleton[bone_index];
			new_v += W(v_index, bone.weight_index) * (T[bone_index] * bone.rest_T
				* Eigen::Vector3d(bone.length, 0, 0));
		}

		U.row(v_index) = new_v.transpose();
	}
  /////////////////////////////////////////////////////////////////////////////
}
