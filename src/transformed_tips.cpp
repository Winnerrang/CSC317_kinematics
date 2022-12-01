
#include "transformed_tips.h"
#include "forward_kinematics.h"
#include <iostream>

Eigen::VectorXd transformed_tips(
  const Skeleton & skeleton, 
  const Eigen::VectorXi & b)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code
	
	std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > T;
	Eigen::VectorXd tip_positions = Eigen::VectorXd::Zero(3 * b.size());
	forward_kinematics(skeleton, T);
	
	for (int i = 0; i < b.size(); i++) {
		//std::cout << skeleton[i].rest_T.matrix() << std::endl <<std::endl;
		auto tip = T[b(i)] * skeleton[b(i)].rest_T * Eigen::Vector3d(skeleton[b(i)].length, 0, 0);
		
		for (int j = 0; j < 3; j++) {
			tip_positions(3 * i + j) = tip(j);
		}
	}

	//std::cout << tip_positions << std::endl;
	
	return tip_positions;
  /////////////////////////////////////////////////////////////////////////////
}
