/*

Copyright (c) 2010, Stephan Weiss, ASL, ETH Zurich, Switzerland
You can contact the author at <stephan dot weiss at ieee dot org>

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

#ifndef MEASUREMENT_H
#define MEASUREMENT_H

#include <msf_core/MSF_Core.h>

namespace msf_core{

class MeasurementHandler;

class Measurements
{
protected:
	typedef std::vector<MeasurementHandler*> Handlers;
	Handlers handlers;

	ReconfigureServer *reconfServer_;

	void Config(msf_core::MSF_CoreConfig &config, uint32_t level);
	virtual void init(double scale) = 0;
	virtual	 void init_scale(double scale) = 0;

public:

	// measurements
	Eigen::Quaternion<double> q_cv_;
	Eigen::Matrix<double, 3, 1> p_vc_;
	Eigen::Matrix<double, 3, 1> v_vc_;
	double press_height_;
	MSF_Core msf_core_;

	void addHandler(MeasurementHandler* handler)
	{
		handlers.push_back(handler);
	}
	Measurements();
	virtual ~Measurements();
};


class MeasurementHandler
{
protected:
	Measurements* measurements;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	MeasurementHandler(Measurements* meas):measurements(meas){}

	virtual ~MeasurementHandler() {}
};

}; // end namespace

#endif /* MEASUREMENT_H */
