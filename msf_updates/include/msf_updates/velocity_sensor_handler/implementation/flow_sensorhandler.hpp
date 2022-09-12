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
#ifndef FLOW_SENSORHANDLER_HPP_
#define FLOW_SENSORHANDLER_HPP_
#include <msf_core/eigen_utils.h>
#include <msf_core/msf_types.h>

namespace msf_velocity_sensor {
template <typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
FlowSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::FlowSensorHandler(
    MANAGER_TYPE& meas, std::string topic_namespace,
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

  subOpticalFlow_ = nh.subscribe<arkflow_ros::OpticalFlow>(
      "optical_flow_input", 20, &FlowSensorHandler::MeasurementCallback, this);
}

template <typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void FlowSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::SetNoises(double n_zv) {
  n_zv_ = n_zv;
}

template <typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void FlowSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::SetDelay(double delay) {
  delay_ = delay;
}

template <typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void FlowSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::ProcessFlowMeasurement(
    const arkflow_ros::OpticalFlowConstPtr& msg) {
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
      fixedstates));

  meas->MakeFromSensorReading(msg, msg->header.stamp.toSec() - delay_);

  z_v_ = meas->_z_v;  // Store this for the init procedure

  this->manager_.msf_core_->AddMeasurement(meas);
}

template <typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void FlowSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::MeasurementCallback(
    const arkflow_ros::OpticalFlowConstPtr& msg) {
  this->SequenceWatchDog(msg->header.seq, subOpticalFlow_.getTopic());
  MSF_INFO_STREAM_ONCE("*** velocity sensor got first measurement from topic "
                       << this->topic_namespace_ << "/"
                       << subOpticalFlow_.getTopic() << " ***");

  ProcessFlowMeasurement(msg);
}

}  // namespace msf_velocity_sensor

#endif  // FLOW_SENSORHANDLER_HPP_