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

#ifndef VELOCITY_MEASUREMENTMANAGER_H_
#define VELOCITY_MEASUREMENTMANAGER_H_

// clang-format off
#include <ros/ros.h>

#include <msf_core/msf_core.h>
#include <msf_core/msf_sensormanagerROS.h>
#include <msf_core/msf_IMUHandler_ROS.h>
#include "msf_statedef.hpp"
#include <msf_updates/velocity_sensor_handler/velocity_sensorhandler.h>
#include <msf_updates/velocity_sensor_handler/velocity_xy_measurement.h>
#include <msf_updates/SingleVelocitySensorConfig.h>
// clang-format on

namespace msf_velocity_sensor {
typedef msf_updates::SingleVelocitySensorConfig Config_T;
typedef dynamic_reconfigure::Server<Config_T> ReconfigureServer;
typedef shared_ptr<ReconfigureServer> ReconfigureServerPtr;

class VelocitySensorManager
    : public msf_core::MSF_SensorManagerROS<msf_updates::EKFState> {
  typedef VelocityXYSensorHandler<
      msf_updates::velocity_xy_measurement::VelocityXYMeasurement<>,
      VelocitySensorManager>
      VelocityXYSensorHandler_T;

  friend class VelocityXYSensorHandler<
      msf_updates::velocity_xy_measurement::VelocityXYMeasurement<>,
      VelocitySensorManager>;

 public:
  typedef msf_updates::EKFState EKFState_T;
  typedef EKFState_T::StateSequence_T StateSequence_T;
  typedef EKFState_T::StateDefinition_T StateDefinition_T;

  VelocitySensorManager(
      ros::NodeHandle pnh = ros::NodeHandle("~/velocity_sensor")) {
    imu_handler_.reset(new msf_core::IMUHandler_ROS<msf_updates::EKFState>(
        *this, "msf_core", "imu_handler"));

    velocity_handler_.reset(
        new VelocityXYSensorHandler_T(*this, "", "velocity_sensor"));
    AddHandler(velocity_handler_);

    reconf_server_.reset(new ReconfigureServer(pnh));
    ReconfigureServer::CallbackType f =
        boost::bind(&VelocitySensorManager::Config, this, _1, _2);
    reconf_server_->setCallback(f);

    // TODO(clanegge): Initialize filter directly, should we make it a service
    // call?
    Init(1.0);
  }

  virtual ~VelocitySensorManager() {}

  virtual const Config_T& Getcfg() { return config_; }

 private:
  shared_ptr<msf_core::IMUHandler_ROS<msf_updates::EKFState>> imu_handler_;
  shared_ptr<VelocityXYSensorHandler_T> velocity_handler_;

  Config_T config_;
  ReconfigureServerPtr reconf_server_;

  /**
   * \brief Dynamic reconfigure callback.
   */
  virtual void Config(Config_T& config, uint32_t level) {
    config_ = config;
    velocity_handler_->SetNoises(config.velocity_noise_meas);
    velocity_handler_->SetDelay(config.velocity_delay);

    if ((level & msf_updates::SingleVelocitySensor_INIT_FILTER) &&
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

    Eigen::Matrix<double, 3, 1> p, v, b_w, b_a, g, w_m, a_m, p_iv;
    Eigen::Matrix<double, 2, 1>
        v_v;  // velocity measured by sensor in velocity sensor frame
    Eigen::Quaternion<double> q, q_iv;
    msf_core::MSF_Core<EKFState_T>::ErrorStateCov P;

    // Init values.
    g << 0., 0., 9.81;  /// Gravity.
    // b_w << 0., 0., 0.;  /// Bias gyroscopes.
    // b_a << 0., 0., 0.;  /// Bias accelerometer.

    v << 0., 0., 0.;    /// Robot velocity (IMU centered).
    w_m << 0., 0., 0.;  /// Initial angular velocity.

    // intial body frame aligned with world frame as its anyway unobservable
    p.setZero();
    // TODO(clanegge): check if world frame is gravity aligned!
    q.setIdentity();

    P.setZero();  // Error state covariance; if zero a default initialization in
                  // msf_core is used.
    v_v = velocity_handler_->GetVelocityMeasurement();
    MSF_INFO_STREAM("initial measurement vel:[" << v_v.transpose() << "]");
    // Check if we have already input from the measurement sensor
    if (!velocity_handler_->ReceivedFirstMeasurement()) {
      MSF_WARN_STREAM(
          "No measurements received yet to initialize velocity - using [0 0 "
          "0]");
      v_v = Eigen::Matrix<double, 2, 1>::Zero();
    }

    ros::NodeHandle pnh("~");
    pnh.param("velocity_sensor/init/b_w/x", b_w[0], 0.0);
    pnh.param("velocity_sensor/init/b_w/y", b_w[1], 0.0);
    pnh.param("velocity_sensor/init/b_w/z", b_w[2], 0.0);

    pnh.param("velocity_sensor/init/b_a/x", b_a[0], 0.0);
    pnh.param("velocity_sensor/init/b_a/y", b_a[1], 0.0);
    pnh.param("velocity_sensor/init/b_a/z", b_a[2], 0.0);

    // Get transformation between velocity sensor and body (IMU) frame
    pnh.param("velocity_sensor/init/p_iv/x", p_iv[0], 0.0);
    pnh.param("velocity_sensor/init/p_iv/y", p_iv[1], 0.0);
    pnh.param("velocity_sensor/init/p_iv/z", p_iv[2], 0.0);

    pnh.param("velocity_sensor/init/q_iv/w", q_iv.w(), 1.0);
    pnh.param("velocity_sensor/init/q_iv/x", q_iv.x(), 0.0);
    pnh.param("velocity_sensor/init/q_iv/y", q_iv.y(), 0.0);
    pnh.param("velocity_sensor/init/q_iv/z", q_iv.z(), 0.0);
    q_iv.normalize();

    // Calculate init velocity based on sensor measurements.
    // 3d state of measured velocity with z value set to zero:
    Eigen::Matrix<double, 3, 1> v_v_3d(v_v[0], v_v[1], 0.0);
    // v = q_iv.toRotationMatrix() * v_v_3d;  // w_m = 0

    a_m = q.inverse() * g;  // Initial acceleration

    // Preapare init "measurement"
    // True means that this message contains initial sensor readings.
    shared_ptr<msf_core::MSF_InitMeasurement<EKFState_T>> meas(
        new msf_core::MSF_InitMeasurement<EKFState_T>(true));

    meas->SetStateInitValue<StateDefinition_T::p>(p);
    meas->SetStateInitValue<StateDefinition_T::v>(v);
    meas->SetStateInitValue<StateDefinition_T::q>(q);
    meas->SetStateInitValue<StateDefinition_T::b_w>(b_w);
    meas->SetStateInitValue<StateDefinition_T::b_a>(b_a);
    meas->SetStateInitValue<StateDefinition_T::q_iv>(q_iv);
    meas->SetStateInitValue<StateDefinition_T::p_iv>(p_iv);

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
        msf_core::Vector3::Constant(config_.velocity_noise_q_iv);
    const msf_core::Vector3 npivv =
        msf_core::Vector3::Constant(config_.velocity_noise_p_iv);

    // Compute the blockwise Q values and store them with the states,
    // these then get copied by the core to the correct places in Qd.
    state.GetQBlock<StateDefinition_T::q_iv>() =
        (dt * nqivv.cwiseProduct(nqivv)).asDiagonal();
    state.GetQBlock<StateDefinition_T::p_iv>() =
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
}  // namespace msf_velocity_sensor

#endif  // VELOCITY_MEASUREMENTMANAGER_H_
