#include "copy_skeleton_at.h"
#include <iostream>

Skeleton copy_skeleton_at(
  const Skeleton & skeleton, 
  const Eigen::VectorXd & A)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code
	Skeleton copy = skeleton;
	
	for (int bone_index = 0; bone_index < skeleton.size(); bone_index++) {
		copy[bone_index].xzx = Eigen::Vector3d(A(bone_index * 3), A(bone_index * 3 + 1), A(bone_index * 3 + 2));
	}
	return copy;
  /////////////////////////////////////////////////////////////////////////////
}
