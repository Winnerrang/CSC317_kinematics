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
	double sigma = max_step;

	double E0 = f(z);


	while (sigma > 1e-4) {
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
