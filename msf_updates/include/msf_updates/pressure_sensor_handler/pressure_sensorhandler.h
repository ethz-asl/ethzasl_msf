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

#ifndef PRESSURE_SENSOR_H
#define PRESSURE_SENSOR_H

#include <msf_core/msf_sensormanagerROS.h>
#include <asctec_hl_comm/mav_imu.h>
#include <msf_updates/pressure_sensor_handler/pressure_measurement.h>
#include <queue>

namespace msf_pressure_sensor{

class PressureSensorHandler : public msf_core::SensorHandler<typename msf_updates::EKFState>
{
private:

  enum{
    heightbuffsize = 10
  };
  Eigen::Matrix<double, 1, 1> z_p_; ///< pressure measurement
  double n_zp_; ///< pressure measurement noise
  Eigen::Matrix<double, 1, 1> z_average_p; ///<averaged pressure measurement
  double heightbuff[heightbuffsize];

  ros::Subscriber subPressure_;

  void measurementCallback(const asctec_hl_comm::mav_imuConstPtr & msg);

public:
  PressureSensorHandler(msf_core::MSF_SensorManager<msf_updates::EKFState>& meas,
                        std::string topic_namespace, std::string parameternamespace);
  //used for the init
  Eigen::Matrix<double, 1, 1> getPressureMeasurement(){
    return z_p_;
  }
  Eigen::Matrix<double, 1, 1> getAveragedPressureMeasurement(){
    return z_average_p;
  }
  //setters for configure values
  void setNoises(double n_zp);

};
}
#include "implementation/pressure_sensorhandler.hpp"

#endif /* POSE_SENSOR_H */
