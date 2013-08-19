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
#ifndef POSE_MEASUREMENTMANAGER_H
#define POSE_MEASUREMENTMANAGER_H

#include <ros/ros.h>

#include <msf_core/msf_core.h>
#include <msf_core/msf_sensormanagerROS.h>
#include <msf_core/msf_IMUHandler_ROS.h>
#include "msf_statedef.hpp"
#include <msf_updates/pose_sensor_handler/pose_sensorhandler.h>
#include <msf_updates/pose_sensor_handler/pose_measurement.h>
#include <msf_updates/SinglePoseSensorConfig.h>

#include "sensor_fusion_comm/InitScale.h"
#include "sensor_fusion_comm/InitHeight.h"

namespace msf_pose_sensor {

typedef msf_updates::SinglePoseSensorConfig Config_T;
typedef dynamic_reconfigure::Server<Config_T> ReconfigureServer;
typedef shared_ptr<ReconfigureServer> ReconfigureServerPtr;

class PoseSensorManager : public msf_core::MSF_SensorManagerROS<
    msf_updates::EKFState> {
  typedef PoseSensorHandler<msf_updates::pose_measurement::PoseMeasurement,
      PoseSensorManager> PoseSensorHandler_T;
  friend class PoseSensorHandler<msf_updates::pose_measurement::PoseMeasurement,
      PoseSensorManager> ;
 public:
  typedef msf_updates::EKFState EKFState_T;
  typedef EKFState_T::StateSequence_T StateSequence_T;
  typedef EKFState_T::StateDefinition_T StateDefinition_T;

  PoseSensorManager(ros::NodeHandle pnh = ros::NodeHandle("~/pose_sensor")) {
    bool distortmeas = false;  ///< Distort the pose measurements.

    imu_handler_.reset(
        new msf_core::IMUHandler_ROS<msf_updates::EKFState>(*this, "msf_core",
                                                            "imu_handler"));
    pose_handler_.reset(
        new PoseSensorHandler_T(*this, "", "pose_sensor", distortmeas));

    addHandler(pose_handler_);

    reconf_server_.reset(new ReconfigureServer(pnh));
    ReconfigureServer::CallbackType f = boost::bind(&PoseSensorManager::config,
                                                    this, _1, _2);
    reconf_server_->setCallback(f);

    init_scale_srv = pnh.advertiseService("initialize_msf_scale",
                                          &PoseSensorManager::init_scale, this);
    init_height_srv = pnh.advertiseService("initialize_msf_height",
                                           &PoseSensorManager::init_height, this);
  }
  virtual ~PoseSensorManager() {
  }

  virtual const Config_T& getcfg() {
    return config_;
  }

 private:
  shared_ptr<msf_core::IMUHandler_ROS<msf_updates::EKFState> > imu_handler_;
  shared_ptr<PoseSensorHandler_T> pose_handler_;

  Config_T config_;
  ReconfigureServerPtr reconf_server_;
  ros::ServiceServer init_scale_srv;
  ros::ServiceServer init_height_srv;

  /**
   * \brief Dynamic reconfigure callback.
   */
  virtual void config(Config_T &config, uint32_t level) {
    config_ = config;
    pose_handler_->setNoises(config.pose_noise_meas_p,
                             config.pose_noise_meas_q);
    pose_handler_->setDelay(config.pose_delay);
    if ((level & msf_updates::SinglePoseSensor_INIT_FILTER)
        && config.core_init_filter == true) {
      init(config.pose_initial_scale);
      config.core_init_filter = false;
    }
    // Init call with "set height" checkbox.
    if ((level & msf_updates::SinglePoseSensor_SET_HEIGHT)
        && config.core_set_height == true) {
      Eigen::Matrix<double, 3, 1> p = pose_handler_->getPositionMeasurement();
      if (p.norm() == 0) {
        MSF_WARN_STREAM(
            "No measurements received yet to initialize position. Height init "
            "not allowed.");
        return;
      }
      double scale = p[2] / config.core_height;
      init(scale);
      config.core_set_height = false;
    }
  }

  bool init_scale(sensor_fusion_comm::InitScale::Request &req,
                  sensor_fusion_comm::InitScale::Response &res) {
    ROS_INFO("Initialize filter with scale %f", req.scale);
    init(req.scale);
    res.result = "Initialized scale";
    return true;
  }

  bool init_height(sensor_fusion_comm::InitHeight::Request &req,
                   sensor_fusion_comm::InitHeight::Response &res) {
    ROS_INFO("Initialize filter with height %f", req.height);
    Eigen::Matrix<double, 3, 1> p = pose_handler_->getPositionMeasurement();
    if (p.norm() == 0) {
      MSF_WARN_STREAM(
          "No measurements received yet to initialize position. Height init "
          "not allowed.");
      return false;
    }
    double scale = p[2] / req.height;
    init(scale);
    res.result = "Initialized height";
    return true;
  }

  void init(double scale) const {
    Eigen::Matrix<double, 3, 1> p, v, b_w, b_a, g, w_m, a_m, p_ic, p_vc, p_wv;
    Eigen::Quaternion<double> q, q_wv, q_ic, q_cv;
    msf_core::MSF_Core<EKFState_T>::ErrorStateCov P;

    // init values
    g << 0, 0, 9.81;	        /// Gravity.
    b_w << 0, 0, 0;		/// Bias gyroscopes.
    b_a << 0, 0, 0;		/// Bias accelerometer.

    v << 0, 0, 0;			/// Robot velocity (IMU centered).
    w_m << 0, 0, 0;		/// Initial angular velocity.
    a_m = g;			/// Initial acceleration.

    q_wv.setIdentity();  // Vision-world rotation drift.
    p_wv.setZero();  // Vision-world position drift.

    P.setZero();  // Error state covariance; if zero, a default initialization in msf_core is used

    p_vc = pose_handler_->getPositionMeasurement();
    q_cv = pose_handler_->getAttitudeMeasurement();

    MSF_INFO_STREAM(
        "initial measurement pos:["<<p_vc.transpose()<<"] orientation: "<<STREAMQUAT(q_cv));

    // Check if we have already input from the measurement sensor.
    if (p_vc.norm() == 0)
      MSF_WARN_STREAM(
          "No measurements received yet to initialize position - using [0 0 0]");
    if (q_cv.w() == 1)
      MSF_WARN_STREAM(
          "No measurements received yet to initialize attitude - using [1 0 0 0]");

    ros::NodeHandle pnh("~");
    pnh.param("pose_sensor/init/p_ic/x", p_ic[0], 0.0);
    pnh.param("pose_sensor/init/p_ic/y", p_ic[1], 0.0);
    pnh.param("pose_sensor/init/p_ic/z", p_ic[2], 0.0);

    pnh.param("pose_sensor/init/q_ic/w", q_ic.w(), 1.0);
    pnh.param("pose_sensor/init/q_ic/x", q_ic.x(), 0.0);
    pnh.param("pose_sensor/init/q_ic/y", q_ic.y(), 0.0);
    pnh.param("pose_sensor/init/q_ic/z", q_ic.z(), 0.0);
    q_ic.normalize();

    // Calculate initial attitude and position based on sensor measurements.
    if (q_cv.w() == 1) {  // If there is no pose measurement, only apply q_wv.
      q = q_wv;
    } else {  // If there is a pose measurement, apply q_ic and q_wv to get initial attitude.
      q = (q_ic * q_cv.conjugate() * q_wv).conjugate();
    }

    q.normalize();
    p = p_wv + q_wv.conjugate().toRotationMatrix() * p_vc / scale
        - q.toRotationMatrix() * p_ic;

    // Prepare init "measurement"
    // True means that this message contains initial sensor readings.
    shared_ptr < msf_core::MSF_InitMeasurement<EKFState_T>
        > meas(new msf_core::MSF_InitMeasurement<EKFState_T>(true));

    meas->setStateInitValue<StateDefinition_T::p>(p);
    meas->setStateInitValue<StateDefinition_T::v>(v);
    meas->setStateInitValue<StateDefinition_T::q>(q);
    meas->setStateInitValue<StateDefinition_T::b_w>(b_w);
    meas->setStateInitValue<StateDefinition_T::b_a>(b_a);
    meas->setStateInitValue<StateDefinition_T::L>(
        Eigen::Matrix<double, 1, 1>::Constant(scale));
    meas->setStateInitValue<StateDefinition_T::q_wv>(q_wv);
    meas->setStateInitValue<StateDefinition_T::p_wv>(p_wv);
    meas->setStateInitValue<StateDefinition_T::q_ic>(q_ic);
    meas->setStateInitValue<StateDefinition_T::p_ic>(p_ic);

    setP(meas->get_P());  // Call my set P function.
    meas->get_w_m() = w_m;
    meas->get_a_m() = a_m;
    meas->time = ros::Time::now().toSec();

    // Call initialization in core.
    msf_core_->init(meas);

  }

  // Prior to this call, all states are initialized to zero/identity.
  virtual void resetState(EKFState_T& state) const {
    //set scale to 1
    Eigen::Matrix<double, 1, 1> scale;
    scale << 1.0;
    state.set < StateDefinition_T::L > (scale);
  }
  virtual void initState(EKFState_T& state) const {
    UNUSED(state);
  }

  virtual void calculateQAuxiliaryStates(EKFState_T& state, double dt) const {
    const msf_core::Vector3 nqwvv = msf_core::Vector3::Constant(
        config_.pose_noise_q_wv);
    const msf_core::Vector3 npwvv = msf_core::Vector3::Constant(
        config_.pose_noise_p_wv);
    const msf_core::Vector3 nqicv = msf_core::Vector3::Constant(
        config_.pose_noise_q_ic);
    const msf_core::Vector3 npicv = msf_core::Vector3::Constant(
        config_.pose_noise_p_ic);
    const msf_core::Vector1 n_L = msf_core::Vector1::Constant(
        config_.pose_noise_scale);

    // Compute the blockwise Q values and store them with the states,
    // these then get copied by the core to the correct places in Qd.
    state.getQBlock<StateDefinition_T::L>() = (dt * n_L.cwiseProduct(n_L))
        .asDiagonal();
    state.getQBlock<StateDefinition_T::q_wv>() =
        (dt * nqwvv.cwiseProduct(nqwvv)).asDiagonal();
    state.getQBlock<StateDefinition_T::p_wv>() =
        (dt * npwvv.cwiseProduct(npwvv)).asDiagonal();
    state.getQBlock<StateDefinition_T::q_ic>() =
        (dt * nqicv.cwiseProduct(nqicv)).asDiagonal();
    state.getQBlock<StateDefinition_T::p_ic>() =
        (dt * npicv.cwiseProduct(npicv)).asDiagonal();
  }

  virtual void setP(
      Eigen::Matrix<double, EKFState_T::nErrorStatesAtCompileTime,
          EKFState_T::nErrorStatesAtCompileTime>& P) const {
    UNUSED(P);
    // Nothing, we only use the simulated cov for the core plus diagonal for the
    // rest.
  }

  virtual void augmentCorrectionVector(
      Eigen::Matrix<double, EKFState_T::nErrorStatesAtCompileTime, 1>& correction) const {
    UNUSED(correction);
  }

  virtual void sanityCheckCorrection(
      EKFState_T& delaystate,
      const EKFState_T& buffstate,
      Eigen::Matrix<double, EKFState_T::nErrorStatesAtCompileTime, 1>& correction) const {
    UNUSED(buffstate);
    UNUSED(correction);

    const EKFState_T& state = delaystate;
    if (state.get<StateDefinition_T::L>()(0) < 0) {
      MSF_WARN_STREAM_THROTTLE(
          1, "Negative scale detected: " << state.get<StateDefinition_T::L>()(0)
          << ". Correcting to 0.1");
      Eigen::Matrix<double, 1, 1> L_;
      L_ << 0.1;
      delaystate.set < StateDefinition_T::L > (L_);
    }
  }
};

}
#endif /* POSE_MEASUREMENTMANAGER_H */
