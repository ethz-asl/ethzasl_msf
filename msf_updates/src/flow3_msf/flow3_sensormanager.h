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

#ifndef FLOW3_MEASUREMENTMANAGER_H_
#define FLOW3_MEASUREMENTMANAGER_H_

// clang-format off
#include <ros/ros.h>

#include <msf_core/msf_core.h>
#include <msf_core/msf_sensormanagerROS.h>
#include <msf_core/msf_IMUHandler_ROS.h>
#include "msf_statedef.hpp"
#include <msf_updates/velocity_sensor_handler/velocity_sensorhandler.h>
#include <msf_updates/flow_sensor_handler/flow_sensorhandler.h>
#include <msf_updates/flow_sensor_handler/flow_measurement.h>
#include <msf_updates/Flow3SensorConfig.h>
// clang-format on

namespace msf_updates {
typedef msf_updates::Flow3SensorConfig Config_T;
typedef dynamic_reconfigure::Server<Config_T> ReconfigureServer;
typedef shared_ptr<ReconfigureServer> ReconfigureServerPtr;

class Flow3SensorManager
    : public msf_core::MSF_SensorManagerROS<msf_updates::EKFState> {
  typedef msf_flow_sensor::FlowSensorHandler<
      msf_updates::flow_measurement::FlowMeasurement<
          EKFState::StateDefinition_T::q_iv_0,
          EKFState::StateDefinition_T::p_iv_0>,
      Flow3SensorManager>
      FlowSensorHandler_0_T;
  typedef msf_flow_sensor::FlowSensorHandler<
      msf_updates::flow_measurement::FlowMeasurement<
          EKFState::StateDefinition_T::q_iv_1,
          EKFState::StateDefinition_T::p_iv_1>,
      Flow3SensorManager>
      FlowSensorHandler_1_T;
  typedef msf_flow_sensor::FlowSensorHandler<
      msf_updates::flow_measurement::FlowMeasurement<
          EKFState::StateDefinition_T::q_iv_2,
          EKFState::StateDefinition_T::p_iv_2>,
      Flow3SensorManager>
      FlowSensorHandler_2_T;

  friend class msf_flow_sensor::FlowSensorHandler<
      msf_updates::flow_measurement::FlowMeasurement<
          EKFState::StateDefinition_T::q_iv_0,
          EKFState::StateDefinition_T::p_iv_0>,
      Flow3SensorManager>;
  friend class msf_flow_sensor::FlowSensorHandler<
      msf_updates::flow_measurement::FlowMeasurement<
          EKFState::StateDefinition_T::q_iv_1,
          EKFState::StateDefinition_T::p_iv_1>,
      Flow3SensorManager>;
  friend class msf_flow_sensor::FlowSensorHandler<
      msf_updates::flow_measurement::FlowMeasurement<
          EKFState::StateDefinition_T::q_iv_2,
          EKFState::StateDefinition_T::p_iv_2>,
      Flow3SensorManager>;

 public:
  typedef msf_updates::EKFState EKFState_T;
  typedef EKFState_T::StateSequence_T StateSequence_T;
  typedef EKFState_T::StateDefinition_T StateDefinition_T;

  Flow3SensorManager(ros::NodeHandle pnh = ros::NodeHandle("~/flow3_sensor")) {
    imu_handler_.reset(new msf_core::IMUHandler_ROS<msf_updates::EKFState>(
        *this, "msf_core", "imu_handler"));

    flow_handler_0_.reset(
        new FlowSensorHandler_0_T(*this, "flow_sensor_0", "flow_sensor_0"));
    AddHandler(flow_handler_0_);
    flow_handler_1_.reset(
        new FlowSensorHandler_1_T(*this, "flow_sensor_1", "flow_sensor_1"));
    AddHandler(flow_handler_1_);
    flow_handler_2_.reset(
        new FlowSensorHandler_2_T(*this, "flow_sensor_2", "flow_sensor_2"));
    AddHandler(flow_handler_2_);

    reconf_server_.reset(new ReconfigureServer(pnh));
    ReconfigureServer::CallbackType f =
        boost::bind(&Flow3SensorManager::Config, this, _1, _2);
    reconf_server_->setCallback(f);

    // Initialize filter directly, scale never changes
    Init(1.0);
  }

  virtual ~Flow3SensorManager() {}

  virtual const Config_T& Getcfg() { return config_; }

 private:
  shared_ptr<msf_core::IMUHandler_ROS<msf_updates::EKFState>> imu_handler_;
  shared_ptr<FlowSensorHandler_0_T> flow_handler_0_;
  shared_ptr<FlowSensorHandler_1_T> flow_handler_1_;
  shared_ptr<FlowSensorHandler_2_T> flow_handler_2_;

  Config_T config_;
  ReconfigureServerPtr reconf_server_;

  /**
   * \brief Dynamic reconfigure callback.
   */
  virtual void Config(Config_T& config, uint32_t level) {
    config_ = config;
    // Using same sensor so setting same config
    flow_handler_0_->SetNoises(config.flow_noise_meas);
    flow_handler_0_->SetDelay(config.flow_delay);
    flow_handler_1_->SetNoises(config.flow_noise_meas);
    flow_handler_1_->SetDelay(config.flow_delay);
    flow_handler_2_->SetNoises(config.flow_noise_meas);
    flow_handler_2_->SetDelay(config.flow_delay);

    if ((level & msf_updates::Flow3Sensor_INIT_FILTER) &&
        config.core_init_filter == true) {
      Init(1.0);
      config.core_init_filter = false;
    }
  }
  void Init(double scale) const {
    if (scale < 0.001) {
      MSF_WARN_STREAM("init scale is " << scale << " correcting to 1.");
      scale = 1;
    } else if (std::abs(1.0 - scale) > 1e-8) {
      // TODO(clanegge): is there any case where the scale is not 1?
      MSF_WARN_STREAM("init scale is " << scale << " and not 1.");
    }

    Eigen::Matrix<double, 3, 1> p, v, b_w, b_a, g, w_m, a_m, p_iv_0, p_iv_1,
        p_iv_2;
    Eigen::Matrix<double, 2, 1>
        v_v;  // velocity measured by sensor in velocity sensor frame
    Eigen::Quaternion<double> q, q_iv_0, q_iv_1, q_iv_2;
    msf_core::MSF_Core<EKFState_T>::ErrorStateCov P;

    // Init values.
    g << 0., 0., 9.81;  /// Gravity.

    v << 0., 0., 0.;    /// Robot velocity (IMU centered).
    w_m << 0., 0., 0.;  /// Initial angular velocity.

    // intial body frame aligned with world frame as its anyway unobservable
    p.setZero();
    // TODO(clanegge): check if world frame is gravity aligned!
    q.setIdentity();

    P.setZero();  // Error state covariance; if zero a default initialization in
                  // msf_core is used.

    // Hack: Assume we always initialize at standstill
    v = Eigen::Matrix<double, 3, 1>::Zero();
    a_m = q.inverse() * g;  // Initial acceleration

    ros::NodeHandle pnh("~");
    // Load initial biases and sensor extrinsics
    pnh.param("flow_sensor/init/b_w/x", b_w[0], 0.0);
    pnh.param("flow_sensor/init/b_w/y", b_w[1], 0.0);
    pnh.param("flow_sensor/init/b_w/z", b_w[2], 0.0);

    pnh.param("flow_sensor/init/b_a/x", b_a[0], 0.0);
    pnh.param("flow_sensor/init/b_a/y", b_a[1], 0.0);
    pnh.param("flow_sensor/init/b_a/z", b_a[2], 0.0);

    // Get transformation between flow sensors and body (IMU) frame
    // First:
    pnh.param("flow_sensor/init/p_iv_0/x", p_iv_0[0], 0.0);
    pnh.param("flow_sensor/init/p_iv_0/y", p_iv_0[1], 0.0);
    pnh.param("flow_sensor/init/p_iv_0/z", p_iv_0[2], 0.0);
    pnh.param("flow_sensor/init/q_iv_0/w", q_iv_0.w(), 1.0);
    pnh.param("flow_sensor/init/q_iv_0/x", q_iv_0.x(), 0.0);
    pnh.param("flow_sensor/init/q_iv_0/y", q_iv_0.y(), 0.0);
    pnh.param("flow_sensor/init/q_iv_0/z", q_iv_0.z(), 0.0);
    q_iv_0.normalize();
    // Second
    pnh.param("flow_sensor/init/p_iv_1/x", p_iv_1[0], 0.0);
    pnh.param("flow_sensor/init/p_iv_1/y", p_iv_1[1], 0.0);
    pnh.param("flow_sensor/init/p_iv_1/z", p_iv_1[2], 0.0);
    pnh.param("flow_sensor/init/q_iv_1/w", q_iv_1.w(), 1.0);
    pnh.param("flow_sensor/init/q_iv_1/x", q_iv_1.x(), 0.0);
    pnh.param("flow_sensor/init/q_iv_1/y", q_iv_1.y(), 0.0);
    pnh.param("flow_sensor/init/q_iv_1/z", q_iv_1.z(), 0.0);
    q_iv_1.normalize();
    // Third:
    pnh.param("flow_sensor/init/p_iv_2/x", p_iv_2[0], 0.0);
    pnh.param("flow_sensor/init/p_iv_2/y", p_iv_2[1], 0.0);
    pnh.param("flow_sensor/init/p_iv_2/z", p_iv_2[2], 0.0);
    pnh.param("flow_sensor/init/q_iv_2/w", q_iv_2.w(), 1.0);
    pnh.param("flow_sensor/init/q_iv_2/x", q_iv_2.x(), 0.0);
    pnh.param("flow_sensor/init/q_iv_2/y", q_iv_2.y(), 0.0);
    pnh.param("flow_sensor/init/q_iv_2/z", q_iv_2.z(), 0.0);
    q_iv_2.normalize();

    // Preapare init "measurement"
    // True means that this message contains initial sensor readings.
    shared_ptr<msf_core::MSF_InitMeasurement<EKFState_T>> meas(
        new msf_core::MSF_InitMeasurement<EKFState_T>(true));

    meas->SetStateInitValue<StateDefinition_T::p>(p);
    meas->SetStateInitValue<StateDefinition_T::v>(v);
    meas->SetStateInitValue<StateDefinition_T::q>(q);
    meas->SetStateInitValue<StateDefinition_T::b_w>(b_w);
    meas->SetStateInitValue<StateDefinition_T::b_a>(b_a);
    meas->SetStateInitValue<StateDefinition_T::q_iv_0>(q_iv_0);
    meas->SetStateInitValue<StateDefinition_T::p_iv_0>(p_iv_0);
    meas->SetStateInitValue<StateDefinition_T::q_iv_1>(q_iv_1);
    meas->SetStateInitValue<StateDefinition_T::p_iv_1>(p_iv_1);
    meas->SetStateInitValue<StateDefinition_T::q_iv_2>(q_iv_2);
    meas->SetStateInitValue<StateDefinition_T::p_iv_2>(p_iv_2);

    SetStateCovariance(meas->GetStateCovariance());  // Call my set P function
    meas->Getw_m() = w_m;
    meas->Geta_m() = a_m;
    meas->time = ros::Time::now().toSec();

    // Call initialization in core.
    msf_core_->Init(meas);
  }

  // Prior to this call, all states are initialized to zero/identity.
  virtual void ResetState(EKFState_T& state) const { UNUSED(state); }

  virtual void InitState(EKFState_T& state) const { UNUSED(state); }

  virtual void CalculateQAuxiliaryStates(EKFState_T& state, double dt) const {
    const msf_core::Vector3 nqivv =
        msf_core::Vector3::Constant(config_.flow_noise_q_iv);
    const msf_core::Vector3 npivv =
        msf_core::Vector3::Constant(config_.flow_noise_p_iv);

    // Compute the blockwise Q values and store them with the states,
    // these then get copied by the core to the correct places in Qd.
    state.GetQBlock<StateDefinition_T::q_iv_0>() =
        (dt * nqivv.cwiseProduct(nqivv)).asDiagonal();
    state.GetQBlock<StateDefinition_T::p_iv_0>() =
        (dt * npivv.cwiseProduct(npivv)).asDiagonal();
    state.GetQBlock<StateDefinition_T::q_iv_1>() =
        (dt * nqivv.cwiseProduct(nqivv)).asDiagonal();
    state.GetQBlock<StateDefinition_T::p_iv_1>() =
        (dt * npivv.cwiseProduct(npivv)).asDiagonal();
    state.GetQBlock<StateDefinition_T::q_iv_2>() =
        (dt * nqivv.cwiseProduct(nqivv)).asDiagonal();
    state.GetQBlock<StateDefinition_T::p_iv_2>() =
        (dt * npivv.cwiseProduct(npivv)).asDiagonal();
  }

  virtual void SetStateCovariance(
      Eigen::Matrix<double, EKFState_T::nErrorStatesAtCompileTime,
                    EKFState_T::nErrorStatesAtCompileTime>& P) const {
    UNUSED(P);
    // Nothing, we only use the simulated cov for the core plus diagonal for the
    // rest.
  }

  virtual void AugmentCorrectionVector(
      Eigen::Matrix<double, EKFState_T::nErrorStatesAtCompileTime, 1>&
          correction) const {
    UNUSED(correction);
  }

  virtual void SanityCheckCorrection(
      EKFState_T& delaystate, const EKFState_T& buffstate,
      Eigen::Matrix<double, EKFState_T::nErrorStatesAtCompileTime, 1>&
          correction) const {
    UNUSED(delaystate);
    UNUSED(buffstate);
    UNUSED(correction);
  }
};
}  // namespace msf_updates

#endif  // VELOCITY_MEASUREMENTMANAGER_H_
