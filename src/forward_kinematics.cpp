#include "forward_kinematics.h"
#include "euler_angles_to_transform.h"
#include <functional> // std::function
#include <iostream>
void forward_kinematics(
  const Skeleton & skeleton,
  std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > & T)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code
	std::function<void(int)> find_orientation;
	
	std::vector<bool> is_assigned(skeleton.size(), false);
	T.resize(skeleton.size(),Eigen::Affine3d::Identity());

	find_orientation = [&](int index) {
		if (is_assigned[index]) return;
		
		auto bone = skeleton[index];
		
		if (bone.parent_index == -1) {
			T[index] = bone.rest_T * euler_angles_to_transform(bone.xzx) *
				bone.rest_T.inverse();
			is_assigned[index] = true;
			return;
		}

		int parent_index = bone.parent_index;
 		if (!is_assigned[parent_index]) find_orientation(parent_index);

		T[index] = T[parent_index] * bone.rest_T * euler_angles_to_transform(bone.xzx) *
			bone.rest_T.inverse();
		is_assigned[index] = true;


	};


	for (int i = 0; i < skeleton.size(); i++) {
		find_orientation(i);
	}

  /////////////////////////////////////////////////////////////////////////////
}
