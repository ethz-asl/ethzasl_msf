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

#include <msf_core/measurement.h>

namespace msf_core{

Measurements::Measurements()
{
	//slynen{
	reconfServer_ = NULL;
	//}
	// setup: initial pos, att, of measurement sensor

	p_vc_ = Eigen::Matrix<double, 3, 1>::Constant(0);
	q_cv_ = Eigen::Quaternion<double>(1, 0, 0, 0);

	press_height_=0;

	msf_core_.registerCallback(&Measurements::Config,this);
}

Measurements::~Measurements()
{
	for (Handlers::iterator it(handlers.begin()); it != handlers.end(); ++it)
		delete *it;

	//slynen{
	if(reconfServer_)
		//}
		delete reconfServer_;
	return;
}


void Measurements::Config(msf_core::MSF_CoreConfig& config, uint32_t level){
	if(level & msf_core::MSF_Core_INIT_FILTER)
	{
		init(config.scale_init);
		config.init_filter = false;
	}
	else if(level & msf_core::MSF_Core_SET_HEIGHT)
	{
		if(p_vc_.norm()==0)
		{
			ROS_WARN_STREAM("No measurements received yet to initialize position - using scale factor " << config.scale_init << " for init");
			init(config.scale_init);
		}
		else
		{
			init(p_vc_[2]/config.height);
			ROS_WARN_STREAM("init filter (set scale to: " << p_vc_[2]/config.height << ")");
		}
		config.set_height = false;
	}
	else if(level & msf_core::MSF_Core_SET_PRESS)
	{
		init_scale(config.scale_init);
		config.set_pressure_height = false;
	}
}

}; // end namespace
