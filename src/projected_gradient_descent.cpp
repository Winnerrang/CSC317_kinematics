#include "projected_gradient_descent.h"
#include "line_search.h"
#include <iostream>

void projected_gradient_descent(
  const std::function<double(const Eigen::VectorXd &)> & f,
  const std::function<Eigen::VectorXd(const Eigen::VectorXd &)> & grad_f,
  const std::function<void(Eigen::VectorXd &)> & proj_z,
  const int max_iters,
  Eigen::VectorXd & z)
{
  /////////////////////////////////////////////////////////////////////////////
  // Add your code here
  
	for (int i = 0; i < max_iters; i++) {
		auto JT = grad_f(z);
		double sigma = line_search(f, proj_z, z, -1 * JT, 10000);
		//std::cout << sigma << std::endl;
		z = z - sigma * JT;
		proj_z(z);
	}
  /////////////////////////////////////////////////////////////////////////////
}
