/*

Copyright (c) 2010, Stephan Weiss, ASL, ETH Zurich, Switzerland
You can contact the author at <stephan dot weiss at ieee dot org>
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

#include "viconpos_sensorhandler.h"
#include <msf_core/eigen_utils.h>
#include <msf_core/msf_sensormanagerROS.hpp>
#include "viconpos_measurement.hpp"

#define N_MEAS 9 // measurement size
ViconPosSensorHandler::ViconPosSensorHandler(msf_core::MSF_SensorManager& meas) :
SensorHandler(meas)
{
	ros::NodeHandle pnh("~");
	pnh.param("measurement_world_sensor", measurement_world_sensor_, true);
	pnh.param("use_fixed_covariance", use_fixed_covariance_, false);

	ROS_INFO_COND(measurement_world_sensor_, "interpreting measurement as sensor w.r.t. world");
	ROS_INFO_COND(!measurement_world_sensor_, "interpreting measurement as world w.r.t. sensor (e.g. ethzasl_ptam)");

	ROS_INFO_COND(use_fixed_covariance_, "using fixed covariance");
	ROS_INFO_COND(!use_fixed_covariance_, "using covariance from sensor");

	subscribe();

	//initialize the measurements this handler provides
	z_p_ = Eigen::Matrix<double, 3, 1>::Constant(0);
}

void ViconPosSensorHandler::subscribe()
{
	ros::NodeHandle nh("msf_update");
	subMeasurement_ = nh.subscribe("pose_measurement", 1, &ViconPosSensorHandler::measurementCallback, this);

	dynamic_cast<msf_core::MSF_SensorManagerROS&>(manager_).registerCallback(&ViconPosSensorHandler::noiseConfig, this);

}

void ViconPosSensorHandler::noiseConfig(msf_core::MSF_CoreConfig& config, uint32_t level)
{
	//	if(level & msf_core::MSF_Core_MISC)
	//	{
	this->n_zp_ = config.meas_noise1;
	//	}
}

void ViconPosSensorHandler::measurementCallback(const geometry_msgs::TransformStampedConstPtr & msg)
{
	//slow down VICON
	if (msg->header.seq%5!=0)
		return;

	boost::shared_ptr<ViconMeasurement> meas( new ViconMeasurement(n_zp_));
	meas->makeFromSensorReading(msg, true, true);

	z_p_ = meas->z_p_; //store this for the init procedure

	this->manager_.msf_core_->addMeasurement(meas);
}
