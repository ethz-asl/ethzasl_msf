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

#ifndef POSITION_SENSOR_H
#define POSITION_SENSOR_H

#include <ssf_core/measurement.h>
#include <ssf_updates/PositionWithCovarianceStamped.h>

class PositionSensorHandler: public ssf_core::MeasurementHandler
{
private:
	// measurements
//  Eigen::Quaternion<double> z_q_; /// attitude measurement camera seen from world - here we do not have an attitude measurement
	Eigen::Matrix<double, 3, 1> z_p_; /// position measurement camera seen from world
	double n_zp_ /*, n_zq_*/; /// position and attitude measurement noise - here we do not have an attitude measurement

	ros::Subscriber subMeasurement_;

	bool measurement_world_sensor_; ///< defines if the pose of the sensor is measured in world coordinates (true, default) or vice versa (false, e.g. PTAM)
	bool use_fixed_covariance_; ///< use fixed covariance set by dynamic reconfigure

	void subscribe();
	void measurementCallback(const ssf_updates::PositionWithCovarianceStampedConstPtr & msg);
	void noiseConfig(ssf_core::SSF_CoreConfig& config, uint32_t level);

public:
	PositionSensorHandler();
	PositionSensorHandler(ssf_core::Measurements* meas);
};

#endif /* POSITION_SENSOR_H */
