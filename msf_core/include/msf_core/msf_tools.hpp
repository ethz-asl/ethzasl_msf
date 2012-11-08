/*
 * msf_tools.hpp
 *
 *  Created on: Nov 8, 2012
 *      Author: slynen
 */

#ifndef MSF_TOOLS_HPP_
#define MSF_TOOLS_HPP_

namespace msf_core{
/// computes the median of a given vector
template<typename T>
double getMedian(const T & data)
{
	BOOST_STATIC_ASSERT_MSG(T::ColsAtCompileTime == 1, "getMedian only takes Eigen column vectors as arguments");
	std::vector<double> mediandistvec;
	mediandistvec.reserve(T::RowsAtCompileTime);
	for (int i = 0; i < T::RowsAtCompileTime; ++i)
		mediandistvec.push_back(data(i));

	if (mediandistvec.size() > 0)
	{
		std::vector<double>::iterator first = mediandistvec.begin();
		std::vector<double>::iterator last = mediandistvec.end();
		std::vector<double>::iterator middle = first + std::floor((last - first) / 2);
		std::nth_element(first, middle, last); // can specify comparator as optional 4th arg
		return *middle;
	}
	else
		return 0;
}
}

#endif /* MSF_TOOLS_HPP_ */
