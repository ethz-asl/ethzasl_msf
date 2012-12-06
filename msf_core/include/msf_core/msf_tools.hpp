/*

Copyright (c) 2012, Simon Lynen, ASL, ETH Zurich, Switzerland
You can contact the author at <slynen at ethz dot ch>

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
* Neither the name of ETHZ-ASL nor the
names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETHZ-ASL BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

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

double timehuman(double val){
	return val - floor(val/1000.)*1000;
}

}

#endif /* MSF_TOOLS_HPP_ */
