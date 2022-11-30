#include "forward_kinematics.h"
#include "euler_angles_to_transform.h"
#include <functional> // std::function

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

		if (skeleton[index].parent_index == -1) {
			T[index] = skeleton[index].rest_T * euler_angles_to_transform(skeleton[index].xzx) *
				skeleton[index].rest_T.inverse();
			is_assigned[index] = true;
			return;
		}

		T[index] = T[skeleton[index].parent_index] * skeleton[index].rest_T * euler_angles_to_transform(skeleton[index].xzx) *
			skeleton[index].rest_T.inverse();
		is_assigned[index] = true;


	};

	for (int i = 0; i < skeleton.size(); i++) {
		find_orientation(i);
	}
  /////////////////////////////////////////////////////////////////////////////
}
