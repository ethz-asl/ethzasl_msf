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
#ifndef SPHERICAL_POSITION_SENSOR_H
#define SPHERICAL_POSITION_SENSOR_H

#include <msf_core/msf_sensormanagerROS.h>
#include <geometry_msgs/PointStamped.h>

namespace msf_spherical_position {

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
class AngleSensorHandler : public msf_core::SensorHandler<
    typename msf_updates::EKFState> {
 private:
  msf_core::Vector2 z_a_;  ///< Angle measurement.
  double n_za_;   ///< Position measurement noise.
  double delay_;       ///< Delay to be subtracted from the ros-timestamp of the
                       // measurement provided by this sensor.

  ros::Subscriber subPointStamped_;
  bool use_fixed_covariance_;
  bool provides_absolute_measurements_;  ///< Does this sensor measure relative or absolute values.

  void MeasurementCallback(const geometry_msgs::PointStampedConstPtr & msg);
 public:
  typedef MEASUREMENT_TYPE measurement_t;
  AngleSensorHandler(MANAGER_TYPE& meas, std::string topic_namespace,
                     std::string parameternamespace);
  // Used for the init.
  msf_core::Vector2 GetAngleMeasurement() {
    return z_a_;
  }
  // Setters for configure values.
  void SetNoises(double n_za);
  void SetDelay(double delay);
};

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
class DistanceSensorHandler : public msf_core::SensorHandler<
    typename msf_updates::EKFState> {
 private:
  msf_core::Vector1 z_d_;  ///< Angle measurement.
  double n_zd_;  ///< Position measurement noise.
  double delay_;        ///< Delay to be subtracted from the ros-timestamp of
                        //the measurement provided by this sensor.

  ros::Subscriber subPointStamped_;

  bool use_fixed_covariance_;
  bool provides_absolute_measurements_;  ///< Does this sensor measure relative or absolute values.

  void MeasurementCallback(const geometry_msgs::PointStampedConstPtr & msg);
 public:
  typedef MEASUREMENT_TYPE measurement_t;
  DistanceSensorHandler(MANAGER_TYPE& meas, std::string topic_namespace,
                        std::string parameternamespace);
  // Used for the init.
  msf_core::Vector1 GetDistanceMeasurement() {
    return z_d_;
  }
  // Setters for configure values.
  void SetNoises(double n_za);
  void SetDelay(double delay);
};
}  // namespace msf_spherical_position
#include "./implementation/spherical_sensorhandler.hpp"

#endif  // SPHERICAL_POSITION_SENSOR_H
