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

#include <msf_core/eigen_utils.h>

namespace msf_pressure_sensor{
PressureSensorHandler::PressureSensorHandler(msf_core::MSF_SensorManager<msf_updates::EKFState>& meas) :
        SensorHandler<msf_updates::EKFState>(meas), n_zp_(1e-6)
{
  ros::NodeHandle pnh("~");

  ros::NodeHandle nh("msf_updates");
  subPressure_ = nh.subscribe<asctec_hl_comm::mav_imu>("pressure_input", 1, &PressureSensorHandler::measurementCallback, this);
 }

void PressureSensorHandler::setNoises(double n_zp)
{
  n_zp_ = n_zp;
}


void PressureSensorHandler::measurementCallback(const asctec_hl_comm::mav_imuConstPtr & msg)
{


  boost::shared_ptr<pressure_measurement::PressureMeasurement> meas( new pressure_measurement::PressureMeasurement(n_zp_));
  meas->makeFromSensorReading(msg, msg->header.stamp.toSec());

  z_p_ = meas->z_p_; //store this for the init procedure

  this->manager_.msf_core_->addMeasurement(meas);
}


}
