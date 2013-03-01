/*

Copyright (c) 2010, Stephan Weiss, ASL, ETH Zurich, Switzerland
You can contact the author at <stephan dot weiss at ieee dot org>
Copyright (c) 2012, Simon Lynen, ASL, ETH Zurich, Switzerland
You can contact the author at <slynen at ethz dot ch>
 Copyright (c) 2012, Markus Achtelik, ASL, ETH Zurich, Switzerland
 You can contact the author at <acmarkus at ethz dot ch>

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

#include <msf_core/eigen_utils.h>

namespace msf_position_sensor{
template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
PositionSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::PositionSensorHandler(MANAGER_TYPE& meas, bool provides_absolute_measurements) :
SensorHandler<msf_updates::EKFState>(meas), n_zp_(1e-6), delay_(0), provides_absolute_measurements_(provides_absolute_measurements)
{
  ros::NodeHandle pnh("~");
  pnh.param("use_fixed_covariance", use_fixed_covariance_, false);

  ROS_INFO_COND(use_fixed_covariance_, "Position sensor is using fixed covariance");
  ROS_INFO_COND(!use_fixed_covariance_, "Position sensor is using covariance from sensor");

  ROS_INFO_COND(provides_absolute_measurements, "Position sensor is handling measurements as absolute values");
  ROS_INFO_COND(!provides_absolute_measurements, "Position sensor is handling measurements as relative values");

  ros::NodeHandle nh("msf_updates");

  subPointStamped_ = nh.subscribe<geometry_msgs::PointStamped>("position_input", 1, &PositionSensorHandler::measurementCallback, this);
  //TODO impl pointwithcov callback

  z_p_.setZero();

}

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void PositionSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::setNoises(double n_zp)
{
  n_zp_ = n_zp;
}

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void PositionSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::setDelay(double delay)
{
  delay_ = delay;
}
template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void PositionSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::measurementCallback(const geometry_msgs::PointStampedConstPtr & msg)
{
  BOOST_STATIC_ASSERT_MSG(msf_updates::EKFState::nStateVarsAtCompileTime < 32, "Your state has more than 32 variables, so check that the fix-states does not overflow"); //do not exceed the 32 bits of int

  boost::shared_ptr<MEASUREMENT_TYPE> meas( new MEASUREMENT_TYPE(n_zp_, use_fixed_covariance_, provides_absolute_measurements_, this->sensorID));

  meas->makeFromSensorReading(msg, msg->header.stamp.toSec() - delay_);

  z_p_ = meas->z_p_; //store this for the init procedure

  this->manager_.msf_core_->addMeasurement(meas);
}


}
