/*
 * Copyright (C) 2012-2013 Simon Lynen, ASL, ETH Zurich, Switzerland
 * You can contact the author at <slynen at ethz dot ch>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef PRESSURE_SENSOR_H
#define PRESSURE_SENSOR_H

#include <queue>

#include <geometry_msgs/PointStamped.h>
#include <msf_core/msf_sensormanagerROS.h>

#include <msf_updates/pressure_sensor_handler/pressure_measurement.h>

namespace msf_pressure_sensor {
class PressureSensorHandler : public msf_core::SensorHandler<
    typename msf_updates::EKFState> {
 private:
  enum {
    heightbuffsize = 10
  };
  Eigen::Matrix<double, 1, 1> z_p_;  ///< Pressure measurement.
  double n_zp_;  ///< Pressure measurement noise.
  Eigen::Matrix<double, 1, 1> z_average_p;  ///<Averaged pressure measurement.
  double heightbuff[heightbuffsize];
  ros::Subscriber subPressure_;
  void MeasurementCallback(const geometry_msgs::PointStampedConstPtr & msg);
 public:
  PressureSensorHandler(
      msf_core::MSF_SensorManager<msf_updates::EKFState>& meas,
      std::string topic_namespace, std::string parameternamespace);
  // Used for the init.
  Eigen::Matrix<double, 1, 1> GetPressureMeasurement() {
    return z_p_;
  }
  Eigen::Matrix<double, 1, 1> GetAveragedPressureMeasurement() {
    return z_average_p;
  }
  // Setters for configure values.
  void SetNoises(double n_zp);
};
}  // namespace msf_pressure_sensor
#include "implementation/pressure_sensorhandler.hpp"
#endif  // POSE_SENSOR_H
