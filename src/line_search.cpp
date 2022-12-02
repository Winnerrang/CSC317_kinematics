#include "line_search.h"
#include <iostream>

double line_search(
  const std::function<double(const Eigen::VectorXd &)> & f,
  const std::function<void(Eigen::VectorXd &)> & proj_z,
  const Eigen::VectorXd & z,
  const Eigen::VectorXd & dz,
  const double max_step)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code
	double sigma = 10000;

	double E0 = f(z);


	for (int iter = 0; iter < max_step; iter++) {
		Eigen::VectorXd test_z = z + sigma * dz;
		proj_z(test_z);
		double E = f(test_z);

		if (E >= E0) {
			sigma /= 2;
		}
		else {
			return sigma;
		}

	}
  return 0;
  /////////////////////////////////////////////////////////////////////////////
}
