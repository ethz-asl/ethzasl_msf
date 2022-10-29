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

#ifndef FLOW3_POSITION_POSE_MEASUREMENTMANAGER_H_
#define FLOW3_POSITION_POSE_MEASUREMENTMANAGER_H_

// clang-format off
#include <ros/ros.h>

#include <msf_core/msf_core.h>
#include <msf_core/msf_sensormanagerROS.h>
#include <msf_core/msf_IMUHandler_ROS.h>
#include "msf_statedef.hpp"
#include <msf_updates/flow_sensor_handler/flow_sensorhandler.h>
#include <msf_updates/flow_sensor_handler/flow_measurement.h>
#include <msf_updates/pose_sensor_handler/pose_sensorhandler.h>
#include <msf_updates/pose_sensor_handler/pose_measurement.h>
#include <msf_updates/position_sensor_handler/position_sensorhandler.h>
#include <msf_updates/position_sensor_handler/position_measurement.h>
#include <msf_updates/Flow3PositionPoseSensorConfig.h>
#include "sensor_fusion_comm/InitScale.h"
// clang-format on

namespace msf_updates {
typedef msf_updates::Flow3PositionPoseSensorConfig Config_T;
typedef dynamic_reconfigure::Server<Config_T> ReconfigureServer;
typedef shared_ptr<ReconfigureServer> ReconfigureServerPtr;

class Flow3PositionPoseSensorManager
    : public msf_core::MSF_SensorManagerROS<msf_updates::EKFState> {
  typedef Flow3PositionPoseSensorManager this_T;

 public:
  typedef msf_updates::EKFState EKFState_T;
  typedef EKFState_T::StateSequence_T StateSequence_T;
  typedef EKFState_T::StateDefinition_T StateDefinition_T;
  // Flow Handlers
  typedef msf_flow_sensor::FlowSensorHandler<
      msf_updates::flow_measurement::FlowMeasurement<
          EKFState::StateDefinition_T::q_iv_0,
          EKFState::StateDefinition_T::p_iv_0>,
      this_T>
      FlowSensorHandler_0_T;
  typedef msf_flow_sensor::FlowSensorHandler<
      msf_updates::flow_measurement::FlowMeasurement<
          EKFState::StateDefinition_T::q_iv_1,
          EKFState::StateDefinition_T::p_iv_1>,
      this_T>
      FlowSensorHandler_1_T;
  typedef msf_flow_sensor::FlowSensorHandler<
      msf_updates::flow_measurement::FlowMeasurement<
          EKFState::StateDefinition_T::q_iv_2,
          EKFState::StateDefinition_T::p_iv_2>,
      this_T>
      FlowSensorHandler_2_T;
  friend class msf_flow_sensor::FlowSensorHandler<
      msf_updates::flow_measurement::FlowMeasurement<
          EKFState::StateDefinition_T::q_iv_0,
          EKFState::StateDefinition_T::p_iv_0>,
      this_T>;
  friend class msf_flow_sensor::FlowSensorHandler<
      msf_updates::flow_measurement::FlowMeasurement<
          EKFState::StateDefinition_T::q_iv_1,
          EKFState::StateDefinition_T::p_iv_1>,
      this_T>;
  friend class msf_flow_sensor::FlowSensorHandler<
      msf_updates::flow_measurement::FlowMeasurement<
          EKFState::StateDefinition_T::q_iv_2,
          EKFState::StateDefinition_T::p_iv_2>,
      this_T>;
  // Pose Handlers
  typedef msf_pose_sensor::PoseSensorHandler<
      msf_updates::pose_measurement::PoseMeasurement<>, this_T>
      PoseSensorHandler_T;
  friend class msf_pose_sensor::PoseSensorHandler<
      msf_updates::pose_measurement::PoseMeasurement<>, this_T>;
  // Position Handlers
  typedef msf_position_sensor::PositionSensorHandler<
      msf_updates::position_measurement::PositionMeasurement, this_T>
      PositionSensorHandler_T;
  friend class msf_position_sensor::PositionSensorHandler<
      msf_updates::position_measurement::PositionMeasurement, this_T>;

  Flow3PositionPoseSensorManager(
      ros::NodeHandle pnh = ros::NodeHandle("~/flow3_position_pose_sensor")) {
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

    pose_handler_.reset(
        new PoseSensorHandler_T(*this, "", "pose_sensor", false));
    AddHandler(pose_handler_);

    position_handler_.reset(
        new PositionSensorHandler_T(*this, "", "position_sensor"));
    AddHandler(position_handler_);

    reconf_server_.reset(new ReconfigureServer(pnh));
    ReconfigureServer::CallbackType f =
        boost::bind(&Flow3PositionPoseSensorManager::Config, this, _1, _2);
    reconf_server_->setCallback(f);

    init_scale_srv_ = pnh.advertiseService("initialize_msf_scale", &Flow3PositionPoseSensorManager::InitScale, this);
  }

  virtual ~Flow3PositionPoseSensorManager() {}

  virtual const Config_T& Getcfg() { return config_; }

 private:
  shared_ptr<msf_core::IMUHandler_ROS<msf_updates::EKFState>> imu_handler_;
  shared_ptr<FlowSensorHandler_0_T> flow_handler_0_;
  shared_ptr<FlowSensorHandler_1_T> flow_handler_1_;
  shared_ptr<FlowSensorHandler_2_T> flow_handler_2_;
  shared_ptr<PoseSensorHandler_T> pose_handler_;
  shared_ptr<PositionSensorHandler_T> position_handler_;

  Config_T config_;
  ReconfigureServerPtr reconf_server_;

  ros::ServiceServer init_scale_srv_;

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

    pose_handler_->SetNoises(config.pose_noise_meas_p,
                             config.pose_noise_meas_q);
    pose_handler_->SetDelay(config.pose_delay);

    position_handler_->SetNoises(config.position_noise_meas);
    position_handler_->SetDelay(config.position_delay);

    if ((level & msf_updates::Flow3PositionPoseSensor_INIT_FILTER) &&
        config.core_init_filter) {
      Init(config.pose_initial_scale);
      config.core_init_filter = false;
    }

    // Init call with "set height" checkbox.
    if ((level & msf_updates::Flow3PositionPoseSensor_SET_HEIGHT) &&
        config.core_set_height) {
      Eigen::Matrix<double, 3, 1> p = pose_handler_->GetPositionMeasurement();
      if (p.norm() == 0) {
        MSF_WARN_STREAM(
            "No measurements received yet to initialize position. Height init "
            "not allowed.");
        return;
      }
      double scale = p[2] / config.core_height;
      Init(scale);
      config.core_set_height = false;
    }
  }

    bool InitScale(sensor_fusion_comm::InitScale::Request &req,
                   sensor_fusion_comm::InitScale::Response &res) {
        ROS_INFO("Initialize filter with scale %f", req.scale);
        Init(req.scale);
        res.result = "Initialized scale";
        return true;
    }

  void Init(double scale) const override {
    if (scale < 0.001) {
      MSF_WARN_STREAM("init scale is " << scale << " correcting to 1.");
      scale = 1;
    } else if (std::abs(1.0 - scale) > 1e-8) {
      // TODO(clanegge): is there any case where the scale is not 1?
      MSF_WARN_STREAM("init scale is " << scale << " and not 1.");
    }

    Eigen::Matrix<double, 3, 1> p, v, b_w, b_a, g, w_m, a_m, p_iv_0, p_iv_1,
        p_iv_2, p_ic, p_vc, p_wv, p_ip, p_pos;
    Eigen::Matrix<double, 2, 1>
        v_v;  // velocity measured by sensor in velocity sensor frame
    Eigen::Quaternion<double> q, q_iv_0, q_iv_1, q_iv_2, q_wv, q_ic, q_vc;
    msf_core::MSF_Core<EKFState_T>::ErrorStateCov P;

    // Init values.
    g << 0., 0., 9.81;  /// Gravity.

    v << 0., 0., 0.;    /// Robot velocity (IMU centered).
    w_m << 0., 0., 0.;  /// Initial angular velocity.

    // initial body frame aligned with world frame as its anyway unobservable
    p.setZero();
    // TODO(clanegge): check if world frame is gravity aligned!
    q.setIdentity();

    P.setZero();  // Error state covariance; if zero a default initialization in
                  // msf_core is used.

    q_wv.setIdentity();  // World-vision rotation drift.
    p_wv.setZero();      // World-vision position drift.

    p_pos = position_handler_->GetPositionMeasurement();

    p_vc = pose_handler_->GetPositionMeasurement();
    q_vc = pose_handler_->GetAttitudeMeasurement();

    // Check if we have already input from the measurement sensor.
    if (!pose_handler_->ReceivedFirstMeasurement())
      MSF_WARN_STREAM(
          "No measurements received yet to initialize vision position and "
          "attitude - "
          "using [0 0 0] and [1 0 0 0] respectively");
    if (!position_handler_->ReceivedFirstMeasurement())
      MSF_WARN_STREAM(
          "No measurements received yet to initialize absolute position - "
          "using [0 0 0]");

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

    pnh.param("pose_sensor/init/p_ic/x", p_ic[0], 0.0);
    pnh.param("pose_sensor/init/p_ic/y", p_ic[1], 0.0);
    pnh.param("pose_sensor/init/p_ic/z", p_ic[2], 0.0);

    pnh.param("pose_sensor/init/q_ic/w", q_ic.w(), 1.0);
    pnh.param("pose_sensor/init/q_ic/x", q_ic.x(), 0.0);
    pnh.param("pose_sensor/init/q_ic/y", q_ic.y(), 0.0);
    pnh.param("pose_sensor/init/q_ic/z", q_ic.z(), 0.0);
    q_ic.normalize();

    pnh.param("position_sensor/init/p_ip/x", p_ip[0], 0.0);
    pnh.param("position_sensor/init/p_ip/y", p_ip[1], 0.0);
    pnh.param("position_sensor/init/p_ip/z", p_ip[2], 0.0);

    MSF_INFO_STREAM("p_iv_0: " << p_iv_0.transpose());
    MSF_INFO_STREAM("q_iv_0: " << STREAMQUAT(q_iv_0));
    MSF_INFO_STREAM("p_iv_1: " << p_iv_1.transpose());
    MSF_INFO_STREAM("q_iv_1: " << STREAMQUAT(q_iv_1));
    MSF_INFO_STREAM("p_iv_2: " << p_iv_2.transpose());
    MSF_INFO_STREAM("q_iv_2: " << STREAMQUAT(q_iv_2));
    MSF_INFO_STREAM("p_ic: " << p_ic.transpose());
    MSF_INFO_STREAM("q_ic: " << STREAMQUAT(q_ic));

    // Calculate initial attitude and position based on sensor measurements
    // here we take the attitude from the pose sensor and augment it with
    // global yaw init.
    double yawinit = config_.position_yaw_init / 180 * M_PI;
    Eigen::Quaterniond yawq(cos(yawinit / 2), 0, 0, sin(yawinit / 2));
    yawq.normalize();

    q = yawq;
    q_wv = q * q_ic * q_vc.conjugate(); //.conjugate();

    MSF_WARN_STREAM("q " << STREAMQUAT(q));
    MSF_WARN_STREAM("q_wv " << STREAMQUAT(q_wv));

    Eigen::Matrix<double, 3, 1> p_vision =
        q_wv.conjugate().toRotationMatrix() * p_vc / scale -
        q.toRotationMatrix() * p_ic;

    // TODO (slynen): what if there is no initial position measurement? Then we
    //  have to shift vision-world later on, before applying the first position
    //  measurement.
    p = p_pos - q.toRotationMatrix() * p_ip;
    p_wv = p - p_vision;  // Shift the vision frame so that it fits the position
    // measurement

    // Prepare init "measurement"
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
    meas->SetStateInitValue<StateDefinition_T::L>(
        Eigen::Matrix<double, 1, 1>::Constant(scale));
    meas->SetStateInitValue<StateDefinition_T::q_wv>(q_wv);
    meas->SetStateInitValue<StateDefinition_T::p_wv>(p_wv);
    meas->SetStateInitValue<StateDefinition_T::q_ic>(q_ic);
    meas->SetStateInitValue<StateDefinition_T::p_ic>(p_ic);
    meas->SetStateInitValue<StateDefinition_T::p_ip>(p_ip);

    SetStateCovariance(meas->GetStateCovariance());  // Call my set P function
    meas->Getw_m() = w_m;
    meas->Geta_m() = a_m;
    meas->time = ros::Time::now().toSec();

    // Call initialization in core.
    msf_core_->Init(meas);
  }

  // Prior to this call, all states are initialized to zero/identity.
  void ResetState(EKFState_T& state) const override {
    // Set scale to 1.
    Eigen::Matrix<double, 1, 1> scale;
    scale << 1.0;
    state.Set<StateDefinition_T::L>(scale);
  }

  virtual void InitState(EKFState_T& state) const { UNUSED(state); }

  virtual void CalculateQAuxiliaryStates(EKFState_T& state, double dt) const {
    const msf_core::Vector3 nqivv =
        msf_core::Vector3::Constant(config_.flow_noise_q_iv);
    const msf_core::Vector3 npivv =
        msf_core::Vector3::Constant(config_.flow_noise_p_iv);

    const msf_core::Vector3 nqwvv =
        msf_core::Vector3::Constant(config_.pose_noise_q_wv);
    const msf_core::Vector3 npwvv =
        msf_core::Vector3::Constant(config_.pose_noise_p_wv);
    const msf_core::Vector3 nqicv =
        msf_core::Vector3::Constant(config_.pose_noise_q_ic);
    const msf_core::Vector3 npicv =
        msf_core::Vector3::Constant(config_.pose_noise_p_ic);
    const msf_core::Vector1 n_L =
        msf_core::Vector1::Constant(config_.pose_noise_scale);

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

    state.GetQBlock<StateDefinition_T::L>() =
        (dt * n_L.cwiseProduct(n_L)).asDiagonal();
    state.GetQBlock<StateDefinition_T::q_wv>() =
        (dt * nqwvv.cwiseProduct(nqwvv)).asDiagonal();
    state.GetQBlock<StateDefinition_T::p_wv>() =
        (dt * npwvv.cwiseProduct(npwvv)).asDiagonal();
    state.GetQBlock<StateDefinition_T::q_ic>() =
        (dt * nqicv.cwiseProduct(nqicv)).asDiagonal();
    state.GetQBlock<StateDefinition_T::p_ic>() =
        (dt * npicv.cwiseProduct(npicv)).asDiagonal();
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
    UNUSED(buffstate);
    UNUSED(correction);

    const EKFState_T& state = delaystate;
    if (state.Get<StateDefinition_T::L>()(0) < 0) {
      MSF_WARN_STREAM_THROTTLE(
          1, "Negative scale detected: " << state.Get<StateDefinition_T::L>()(0)
                                         << ". Correcting to 0.1");
      Eigen::Matrix<double, 1, 1> L_;
      L_ << 0.1;
      delaystate.Set<StateDefinition_T::L>(L_);
    }
  }
};
}  // namespace msf_updates

#endif  // FLOW3_POSITION_POSE_MEASUREMENTMANAGER_H_
