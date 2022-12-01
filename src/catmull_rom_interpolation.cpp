#include "catmull_rom_interpolation.h"
#include <Eigen/Dense>
#include <assert.h>
#include <iostream>

Eigen::Vector3d catmull_rom_interpolation(
  const std::vector<std::pair<double, Eigen::Vector3d> > & keyframes,
  double t)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code
	if (keyframes.size() < 2)  return Eigen::Vector3d::Zero();

	//binary search to find which two data the time is in between
	int l(0), r(keyframes.size() - 1);
	
	// it is doing extrapolation, assume constant 
	if (keyframes[l].first >= t) {

		return keyframes[l].second;
	}
	else if (keyframes[r].first <= t) {
		return keyframes[r].second;
	}

	while(r - l > 1){
	/*	std::cout << l << std::endl;
		std::cout << r << std::endl;*/
		int new_index = (l + r) / 2;
		double new_t = keyframes[new_index].first;
		if (new_t < t) l = new_index;
		else if (new_t > t) r = new_index;
		else return keyframes[new_index].second;

	}

	assert(keyframes[l].first <= t && keyframes[r].first >= t);
	//std::cout << l  << " " << r << std::endl;
	
	assert(r - l >= 0 && r - l <= 1);
	if (keyframes[l].first == t) return keyframes[l].second;
	if (keyframes[r].first == t) return keyframes[r].second;

	/*if (l <= 0 || r >= keyframes.size() - 1) {
		std::cout << keyframes.size() << " " << l << " " << r  << std::endl;
		std::cout << keyframes[l].first << std::endl;
		std::cout << keyframes[r].first << std::endl;
		std::cout << t << std::endl;
		std::cout << "The t is in the first or last segement" << std::endl;
		exit(1);
	}*/

	
	auto p1(keyframes[l]), p2(keyframes[r]);
	
	// convert the t to delta that range between 0 and 1
	auto delta = (t - p1.first) / (p2.first - p1.first);

	// Hermite function
	double H0 = 2 * pow(delta, 3.0) - 3 * pow(delta, 2.0) + 1;
	double H1 = -2 * pow(delta, 3.0) + 3 * pow(delta, 2.0);
	double H2 = pow(delta, 3.0) - 2 * pow(delta, 2.0) + delta;
	double H3 = pow(delta, 3.0) - pow(delta, 2.0);
	
	

	auto y1 = p1.second;
	auto y2 = p2.second;

	auto m1 = (l == 0)? Eigen::Vector3d(0, 0, 0) :
		0.5 / (p2.first - keyframes[l-1].first) * (p2.second - keyframes[l-1].second);
	auto m2 = (r == keyframes.size() - 1) ? Eigen::Vector3d(0, 0, 0):
		0.5 / (keyframes[r + 1].first - p1.first) * (keyframes[r + 1].second - p1.second);
  
	return H0 * y1 + H1 * y2 + H2 * m1 + H3 * m2;
  /////////////////////////////////////////////////////////////////////////////
}
