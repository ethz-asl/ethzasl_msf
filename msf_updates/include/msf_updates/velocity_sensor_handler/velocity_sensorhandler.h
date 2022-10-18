/*
 * Copyright (C) 2012-2013 Simon Lynen, ASL, ETH Zurich, Switzerland
 * You can contact the author at <slynen at ethz dot ch>
 * Copyright (C) 2011-2012 Stephan Weiss, ASL, ETH Zurich, Switzerland
 * You can contact the author at <stephan dot weiss at ieee dot org>
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
#ifndef VELOCITY_SENSORHANDLER_H_
#define VELOCITY_SENSORHANDLER_H_

#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <msf_core/msf_sensormanagerROS.h>
#include <nav_msgs/Odometry.h>

namespace msf_velocity_sensor {

template <typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
class VelocityXYSensorHandler
    : public msf_core::SensorHandler<typename msf_updates::EKFState> {
 private:
  Eigen::Matrix<double, 2, 1> z_v_{
      Eigen::Matrix<double, 2, 1>::Zero()};  ///< Velocity measurement.
  double n_zv_{0.0};                         ///< Velocity measurement noise.
  double delay_{0.0};  ///< Delay to be subtracted from the ros-timestamp of the
                       ///< measurement provided by this sensor
  bool use_fixed_covariance_{
      true};  ///< Use fixed covariance set by dynamic reconfigure
  bool provides_absolute_measurements_{
      true};  ///< Does this sensor measure relative or absolute values

  ros::Subscriber subTwistWCovarianceStamped_;
  ros::Subscriber subOdometry_;

  void ProcessVelocityMeasurement(
      const geometry_msgs::TwistWithCovarianceStampedConstPtr& msg);
  void MeasurementCallback(
      const geometry_msgs::TwistWithCovarianceStampedConstPtr& msg);
  void ProcessVelocityMeasurement(const nav_msgs::OdometryConstPtr& msg);
  void MeasurementCallback(const nav_msgs::OdometryConstPtr& msg);

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef MEASUREMENT_TYPE measurement_t;

  // TODO(clanegge): Find a better way of how to do this
  // Store sensor specific transformation from imu to sensor (needed for multi
  // sensor setup)
  Eigen::Matrix<double, 3, 3> C_vi_{Eigen::Matrix<double, 3, 3>::Identity()};
  Eigen::Matrix<double, 3, 1> p_iv_{Eigen::Vector3d::Zero()};

  VelocityXYSensorHandler(MANAGER_TYPE& meas, std::string topic_namespace,
                          std::string parameternamespace);

  // Used for the init.
  Eigen::Matrix<double, 2, 1> GetVelocityMeasurement() { return z_v_; }
  // Setters to configure values.
  void SetNoises(double n_zv);
  void SetDelay(double delay);
};
}  // namespace msf_velocity_sensor

#include "implementation/velocity_sensorhandler.hpp"

#endif  // VELOCITY_SENSORHANDLER_H_