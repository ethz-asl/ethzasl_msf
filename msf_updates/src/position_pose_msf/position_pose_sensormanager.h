/*

 Copyright (c) 2013, Simon Lynen, ASL, ETH Zurich, Switzerland
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

#ifndef POSITION_POSE_SENSOR_MANAGER_H
#define POSITION_POSE_SENSOR_MANAGER_H

#include <ros/ros.h>

#include <msf_core/msf_core.h>
#include <msf_core/msf_sensormanagerROS.h>
#include <msf_core/msf_IMUHandler_ROS.h>
#include "msf_statedef.hpp"
#include <msf_updates/pose_sensor_handler/pose_sensorhandler.h>
#include <msf_updates/pose_sensor_handler/pose_measurement.h>
#include <msf_updates/position_sensor_handler/position_sensorhandler.h>
#include <msf_updates/position_sensor_handler/position_measurement.h>
#include <msf_updates/PositionPoseSensorConfig.h>

namespace msf_updates {

typedef msf_updates::PositionPoseSensorConfig Config_T;
typedef dynamic_reconfigure::Server<Config_T> ReconfigureServer;
typedef shared_ptr<ReconfigureServer> ReconfigureServerPtr;

class PositionPoseSensorManager : public msf_core::MSF_SensorManagerROS<
    msf_updates::EKFState> {
  typedef PositionPoseSensorManager this_T;
  typedef msf_pose_sensor::PoseSensorHandler<
      msf_updates::pose_measurement::PoseMeasurement, this_T> PoseSensorHandler_T;
  friend class msf_pose_sensor::PoseSensorHandler<
      msf_updates::pose_measurement::PoseMeasurement, this_T>;
  typedef msf_position_sensor::PositionSensorHandler<
      msf_updates::position_measurement::PositionMeasurement, this_T> PositionSensorHandler_T;
  friend class msf_position_sensor::PositionSensorHandler<
      msf_updates::position_measurement::PositionMeasurement, this_T>;
 public:
  typedef msf_updates::EKFState EKFState_T;
  typedef EKFState_T::StateSequence_T StateSequence_T;
  typedef EKFState_T::StateDefinition_T StateDefinition_T;

  PositionPoseSensorManager(
      ros::NodeHandle pnh = ros::NodeHandle("~/position_pose_sensor")) {

    imu_handler_.reset(new msf_core::IMUHandler_ROS<msf_updates::EKFState>(*this, "msf_core", "imu_handler"));

    bool distortmeas = false;  ///<distort the pose measurements TODO make param

    pose_handler_.reset(
        new PoseSensorHandler_T(*this, "", "pose_sensor", distortmeas));
    addHandler(pose_handler_);

    position_handler_.reset(
        new PositionSensorHandler_T(*this, "", "position_sensor"));
    addHandler(position_handler_);

    reconf_server_.reset(new ReconfigureServer(pnh));
    ReconfigureServer::CallbackType f = boost::bind(&this_T::config, this, _1,
                                                    _2);
    reconf_server_->setCallback(f);
  }
  virtual ~PositionPoseSensorManager() {
  }

  virtual const Config_T& getcfg() {
    return config_;
  }

 private:
  shared_ptr<msf_core::IMUHandler_ROS<msf_updates::EKFState> > imu_handler_;
  shared_ptr<PoseSensorHandler_T> pose_handler_;
  shared_ptr<PositionSensorHandler_T> position_handler_;

  Config_T config_;
  ReconfigureServerPtr reconf_server_;  ///< dynamic reconfigure server

  /**
   * \brief dynamic reconfigure callback
   */
  virtual void config(Config_T &config, uint32_t level) {
    config_ = config;
    pose_handler_->setNoises(config.pose_noise_meas_p,
                             config.pose_noise_meas_q);
    pose_handler_->setDelay(config.pose_delay);

    position_handler_->setNoises(config.position_noise_meas);
    position_handler_->setDelay(config.position_delay);

    if ((level & msf_updates::PositionPoseSensor_INIT_FILTER)
        && config.core_init_filter == true) {
      init(config.pose_initial_scale);
      config.core_init_filter = false;
    }

    //Init call with "set height" checkbox
    if ((level & msf_updates::PositionPoseSensor_SET_HEIGHT)
        && config.core_set_height == true) {
      Eigen::Matrix<double, 3, 1> p = pose_handler_->getPositionMeasurement();
      if (p.norm() == 0) {
        MSF_WARN_STREAM(
            "No measurements received yet to initialize position. Height init not allowed.");
        return;
      }
      double scale = p[2] / config.core_height;
      init(scale);
      config.core_set_height = false;
    }
  }

  void init(double scale) const {
    Eigen::Matrix<double, 3, 1> p, v, b_w, b_a, g, w_m, a_m, p_ic, p_vc, p_wv,
        p_ip, p_pos;
    Eigen::Quaternion<double> q, q_wv, q_ic, q_vc;
    msf_core::MSF_Core<EKFState_T>::ErrorStateCov P;

    // init values
    g << 0, 0, 9.81;	        /// gravity
    b_w << 0, 0, 0;		/// bias gyroscopes
    b_a << 0, 0, 0;		/// bias accelerometer

    v << 0, 0, 0;			/// robot velocity (IMU centered)
    w_m << 0, 0, 0;		/// initial angular velocity
    a_m = g;			/// initial acceleration

    q_wv.setIdentity();  // world-vision rotation drift
    p_wv.setZero();  //world-vision position drift

    P.setZero();  // error state covariance; if zero, a default initialization in msf_core is used

    p_pos = position_handler_->getPositionMeasurement();

    p_vc = pose_handler_->getPositionMeasurement();
    q_vc = pose_handler_->getAttitudeMeasurement();

    MSF_INFO_STREAM(
        "initial measurement vision: pos:["<<p_vc.transpose()<<"] orientation: "<<STREAMQUAT(q_vc));
    MSF_INFO_STREAM(
        "initial measurement position: pos:["<<p_pos.transpose()<<"]");

    // check if we have already input from the measurement sensor
    if (p_vc.norm() == 0)
      MSF_WARN_STREAM(
          "No measurements received yet to initialize vision position - using [0 0 0]");
    if (p_pos.norm() == 0)
      MSF_WARN_STREAM(
          "No measurements received yet to initialize absolute position - using [0 0 0]");
    if (q_vc.w() == 1)
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

    MSF_INFO_STREAM("p_ic: "<<p_ic.transpose());
    MSF_INFO_STREAM("q_ic: "<<STREAMQUAT(q_ic));

    pnh.param("position_sensor/init/p_ip/x", p_ip[0], 0.0);
    pnh.param("position_sensor/init/p_ip/y", p_ip[1], 0.0);
    pnh.param("position_sensor/init/p_ip/z", p_ip[2], 0.0);

    // calculate initial attitude and position based on sensor measurements
    //here we take the attitude from the pose sensor and augment it with global yaw init
    double yawinit = config_.position_yaw_init / 180 * M_PI;
    Eigen::Quaterniond yawq(cos(yawinit / 2), 0, 0, sin(yawinit / 2));
    yawq.normalize();

    q = yawq;
    q_wv = (q * q_ic * q_vc.conjugate()).conjugate();

    MSF_WARN_STREAM("q "<<STREAMQUAT(q));
    MSF_WARN_STREAM("q_wv "<<STREAMQUAT(q_wv));

    Eigen::Matrix<double, 3, 1> p_vision = q_wv.conjugate().toRotationMatrix()
        * p_vc / scale - q.toRotationMatrix() * p_ic;

    //TODO what if there is no initial position measurement? Then we have to shift vision-world later on, before
    //applying the first position measurement
    p = p_pos - q.toRotationMatrix() * p_ip;
    p_wv = p - p_vision;  //shift the vision frame so that it fits the position measurement

    //we want z from vision (we did scale init), so:
//    p(2) = p_vision(2);
//    p_wv(2) = 0;
//    position_handler_->adjustGPSZReference(p(2));

    //prepare init "measurement"
    shared_ptr<msf_core::MSF_InitMeasurement<EKFState_T> > meas(
        new msf_core::MSF_InitMeasurement<EKFState_T>(true));  //hand over that we will also set the sensor readings

    meas->setStateInitValue < StateDefinition_T::p > (p);
    meas->setStateInitValue < StateDefinition_T::v > (v);
    meas->setStateInitValue < StateDefinition_T::q > (q);
    meas->setStateInitValue < StateDefinition_T::b_w > (b_w);
    meas->setStateInitValue < StateDefinition_T::b_a > (b_a);
    meas->setStateInitValue < StateDefinition_T::L
        > (Eigen::Matrix<double, 1, 1>::Constant(scale));
    meas->setStateInitValue < StateDefinition_T::q_wv > (q_wv);
    meas->setStateInitValue < StateDefinition_T::p_wv > (p_wv);
    meas->setStateInitValue < StateDefinition_T::q_ic > (q_ic);
    meas->setStateInitValue < StateDefinition_T::p_ic > (p_ic);
    meas->setStateInitValue < StateDefinition_T::p_ip > (p_ip);

    setP(meas->get_P());  //call my set P function
    meas->get_w_m() = w_m;
    meas->get_a_m() = a_m;
    meas->time = ros::Time::now().toSec();

    // call initialization in core
    msf_core_->init(meas);
  }

  //prior to this call, all states are initialized to zero/identity
  virtual void resetState(EKFState_T& state) const {
    //set scale to 1

    Eigen::Matrix<double, 1, 1> scale;
    scale << 1.0;
    state.set<StateDefinition_T::L>(scale);
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

    //compute the blockwise Q values and store them with the states,
    //these then get copied by the core to the correct places in Qd
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
    //nothing, we only use the simulated cov for the core plus diagonal for the rest
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
          1,
          "Negative scale detected: " << state.get<StateDefinition_T::L>()(0) << ". Correcting to 0.1");
      Eigen::Matrix<double, 1, 1> L_;
      L_ << 0.1;
      delaystate.set<StateDefinition_T::L>(L_);
    }

  }

};

}
#endif /* POSITION_POSE_SENSOR_MANAGER_H */
