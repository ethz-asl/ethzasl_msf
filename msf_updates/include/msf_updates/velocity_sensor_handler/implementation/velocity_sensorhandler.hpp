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
#ifndef VELOCITY_SENSORHANDLER_HPP_
#define VELOCITY_SENSORHANDLER_HPP_
#include <msf_core/eigen_utils.h>
#include <msf_core/msf_types.h>

namespace msf_velocity_sensor {
template <typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
VelocityXYSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::
    VelocityXYSensorHandler(MANAGER_TYPE& meas, std::string topic_namespace,
                            std::string parameternamespace)
    : SensorHandler<msf_updates::EKFState>(meas, topic_namespace,
                                           parameternamespace),
      z_v_(Eigen::Matrix<double, 2, 1>::Zero()),
      n_zv_(1e-6),  // TODO(clanegge): What should the value of this be?
      delay_(0.0) {
  ros::NodeHandle pnh("~/" + parameternamespace);

  MSF_INFO_STREAM("Loading parameters for velocity sensor from namespace: "
                  << pnh.getNamespace());

  pnh.param("velocity_absolute_measurements", provides_absolute_measurements_,
            true);
  pnh.param("use_fixed_covariance", use_fixed_covariance_, false);
  // pnh.param("velocity_measurement_minimum_dt",
  // velocity_measurement_minimum_dt_, 0.05);
  pnh.param("enable_mah_outlier_rejection", enable_mah_outlier_rejection_,
            false);
  pnh.param("mah_threshold", mah_threshold_,
            msf_core::kDefaultMahThreshold_ / 10.0);

  // Load sensor transform:
  double qw, qx, qy, qz, x, y, z;
  pnh.param("init/q_iv/w", qw, 1.0);
  pnh.param("init/q_iv/x", qx, 0.0);
  pnh.param("init/q_iv/y", qy, 0.0);
  pnh.param("init/q_iv/z", qz, 0.0);

  pnh.param("init/p_iv/x", x, 0.0);
  pnh.param("init/p_iv/y", y, 0.0);
  pnh.param("init/p_iv/z", z, 0.0);

  Eigen::Quaterniond q_iv = Eigen::Quaternion<double>(qw, qx, qy, qz);
  q_iv.normalize();
  C_vi_ = q_iv.inverse().toRotationMatrix();
  p_iv_ = Eigen::Vector3d(x, y, z);

  MSF_INFO_STREAM_COND(provides_absolute_measurements_,
                       "Velocity sensor is handling "
                       "measurements as absolute values");
  MSF_INFO_STREAM_COND(!provides_absolute_measurements_,
                       "Velocity sensor is "
                       "handling measurements as relative values");
  MSF_INFO_STREAM_COND(use_fixed_covariance_,
                       "Velocity sensor is using fixed "
                       "covariance");
  MSF_INFO_STREAM_COND(!use_fixed_covariance_,
                       "Velocity sensor is using covariance "
                       "from sensor");

  ros::NodeHandle nh("msf_updates/" + topic_namespace);
  subTwistWCovarianceStamped_ =
      nh.subscribe<geometry_msgs::TwistWithCovarianceStamped>(
          "twist_with_covariance_stamped_input", 20,
          &VelocityXYSensorHandler::MeasurementCallback, this);
  subOdometry_ = nh.subscribe<nav_msgs::Odometry>(
      "odometry_input", 20, &VelocityXYSensorHandler::MeasurementCallback,
      this);
}

template <typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void VelocityXYSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::SetNoises(
    double n_zv) {
  n_zv_ = n_zv;
}

template <typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void VelocityXYSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::SetDelay(
    double delay) {
  delay_ = delay;
}

template <typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void VelocityXYSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::
    ProcessVelocityMeasurement(const nav_msgs::OdometryConstPtr& msg) {
  received_first_measurement_ = true;

  // Convert odometry message to twist with covariance stamped
  // geometry_msgs::TwistWithCovarianceStampedPtr twist_msg =
  // geometry_msgs::TwistWithCovarianceStampedPtr();
  auto twist_msg(boost::make_shared<geometry_msgs::TwistWithCovarianceStamped>(
      geometry_msgs::TwistWithCovarianceStamped()));

  twist_msg->header = msg->header;
  twist_msg->twist = msg->twist;

  // Get the fixed states
  int fixedstates = 0;
  static_assert(
      msf_updates::EKFState::nStateVarsAtCompileTime < 32,
      "Your state "
      "has more than 32 variables. The code needs to be changed here to have a "
      "larger variable to mark the fixed_states");
  // Do not exceed the 32 bits of int.

  // Get all the fixed states and set flag bits
  MANAGER_TYPE* mngr = dynamic_cast<MANAGER_TYPE*>(&manager_);

  if (mngr) {
    if (mngr->Getcfg().velocity_fixed_p_iv) {
      fixedstates |= 1 << MEASUREMENT_TYPE::AuxState::p_iv;
    } else {
      // TODO(clanegge):
      MSF_ERROR_STREAM(
          "Online calibration of velocity sensor not implemented yet. Using "
          "fixed position.");
      fixedstates |= 1 << MEASUREMENT_TYPE::AuxState::p_iv;
    }
    if (mngr->Getcfg().velocity_fixed_q_iv) {
      fixedstates |= 1 << MEASUREMENT_TYPE::AuxState::q_iv;
    } else {
      // TODO(clanegge):
      MSF_ERROR_STREAM(
          "Online calibration of velocity sensor not implemented yet. Using "
          "fixed rotation.");
      fixedstates |= 1 << MEASUREMENT_TYPE::AuxState::q_iv;
    }
  }

  shared_ptr<MEASUREMENT_TYPE> meas(new MEASUREMENT_TYPE(
      n_zv_, use_fixed_covariance_, provides_absolute_measurements_,
      this->sensorID, enable_mah_outlier_rejection_, mah_threshold_,
      fixedstates, C_vi_, p_iv_));

  meas->MakeFromSensorReading(twist_msg, twist_msg->header.stamp.toSec() -
  delay_);

  z_v_ = meas->_z_v;  // Store this for the init procedure

  this->manager_.msf_core_->AddMeasurement(meas);
}

template <typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void VelocityXYSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::
    ProcessVelocityMeasurement(
        const geometry_msgs::TwistWithCovarianceStampedConstPtr& msg) {
  received_first_measurement_ = true;

  // Get the fixed states.
  int fixedstates = 0;
  static_assert(
      msf_updates::EKFState::nStateVarsAtCompileTime < 32,
      "Your state "
      "has more than 32 variables. The code needs to be changed here to have a "
      "larger variable to mark the fixed_states");
  // Do not exceed the 32 bits of int.

  // Get all the fixed states and set flag bits
  MANAGER_TYPE* mngr = dynamic_cast<MANAGER_TYPE*>(&manager_);

  if (mngr) {
    if (mngr->Getcfg().velocity_fixed_p_iv) {
      fixedstates |= 1 << MEASUREMENT_TYPE::AuxState::p_iv;
    } else {
      // TODO(clanegge):
      MSF_ERROR_STREAM(
          "Online calibration of velocity sensor not implemented yet. Using "
          "fixed position.");
      fixedstates |= 1 << MEASUREMENT_TYPE::AuxState::p_iv;
    }
    if (mngr->Getcfg().velocity_fixed_q_iv) {
      fixedstates |= 1 << MEASUREMENT_TYPE::AuxState::q_iv;
    } else {
      // TODO(clanegge):
      MSF_ERROR_STREAM(
          "Online calibration of velocity sensor not implemented yet. Using "
          "fixed rotation.");
      fixedstates |= 1 << MEASUREMENT_TYPE::AuxState::q_iv;
    }
  }

  shared_ptr<MEASUREMENT_TYPE> meas(new MEASUREMENT_TYPE(
      n_zv_, use_fixed_covariance_, provides_absolute_measurements_,
      this->sensorID, enable_mah_outlier_rejection_, mah_threshold_,
      fixedstates, C_vi_, p_iv_));

  meas->MakeFromSensorReading(msg, msg->header.stamp.toSec() - delay_);

  z_v_ = meas->_z_v;  // Store this for the init procedure

  this->manager_.msf_core_->AddMeasurement(meas);
}

template <typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void VelocityXYSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::
    MeasurementCallback(
        const geometry_msgs::TwistWithCovarianceStampedConstPtr& msg) {
  this->SequenceWatchDog(msg->header.seq,
                         subTwistWCovarianceStamped_.getTopic());
  MSF_INFO_STREAM_ONCE("*** velocity sensor got first measurement from topic "
                       << this->topic_namespace_ << "/"
                       << subTwistWCovarianceStamped_.getTopic() << " ***");

  ProcessVelocityMeasurement(msg);
}

template <typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void VelocityXYSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::
    MeasurementCallback(const nav_msgs::OdometryConstPtr& msg) {
  this->SequenceWatchDog(msg->header.seq, subOdometry_.getTopic());
  MSF_INFO_STREAM_ONCE("*** velocity sensor got first measurement from topic "
                       << this->topic_namespace_ << "/"
                       << subOdometry_.getTopic() << " ***");

  ProcessVelocityMeasurement(msg);
}

}  // namespace msf_velocity_sensor

#endif  // VELOCITY_SENSORHANDLER_HPP_