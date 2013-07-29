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
#ifndef SPHERICAL_POSITION_MEASUREMENTMANAGER_H
#define SPHERICAL_POSITION_MEASUREMENTMANAGER_H

#include <ros/ros.h>

#include <msf_core/msf_core.h>
#include <msf_core/msf_sensormanagerROS.h>
#include "msf_statedef.hpp"
#include <msf_updates/spherical_position_sensor/spherical_sensorhandler.h>
#include <msf_updates/spherical_position_sensor/spherical_measurement.h>
#include <msf_updates/SphericalPositionSensorConfig.h>

namespace msf_spherical_position {

typedef msf_updates::SphericalPositionSensorConfig Config_T;
typedef dynamic_reconfigure::Server<Config_T> ReconfigureServer;
typedef boost::shared_ptr<ReconfigureServer> ReconfigureServerPtr;

class SensorManager : public msf_core::MSF_SensorManagerROS<
    msf_updates::EKFState> {
  typedef AngleSensorHandler<AngleMeasurement, SensorManager> AngleSensorHandler_T;
  friend class AngleSensorHandler<AngleMeasurement, SensorManager> ;
  typedef DistanceSensorHandler<DistanceMeasurement, SensorManager> DistanceSensorHandler_T;
  friend class DistanceSensorHandler<DistanceMeasurement, SensorManager> ;
 public:
  typedef msf_updates::EKFState EKFState_T;
  typedef EKFState_T::StateSequence_T StateSequence_T;
  typedef EKFState_T::StateDefinition_T StateDefinition_T;

  SensorManager(
      ros::NodeHandle pnh = ros::NodeHandle("~/spherical_position_sensor")) {
    angle_handler_.reset(
        new AngleSensorHandler_T(*this, "", "spherical_position_sensor"));
    addHandler(angle_handler_);
    distance_handler_.reset(
        new DistanceSensorHandler_T(*this, "", "spherical_position_sensor"));
    addHandler(distance_handler_);

    reconf_server_.reset(new ReconfigureServer(pnh));
    ReconfigureServer::CallbackType f = boost::bind(&SensorManager::config,
                                                    this, _1, _2);
    reconf_server_->setCallback(f);
  }
  virtual ~SensorManager() {
  }

  virtual const Config_T& getcfg() const {
    return config_;
  }

 private:
  boost::shared_ptr<AngleSensorHandler_T> angle_handler_;
  boost::shared_ptr<DistanceSensorHandler_T> distance_handler_;

  Config_T config_;
  ReconfigureServerPtr reconf_server_;

  /**
   * \brief Dynamic reconfigure callback.
   */
  virtual void config(Config_T &config, uint32_t level) {
    config_ = config;
    angle_handler_->setNoises(config.angle_noise_meas);
    angle_handler_->setDelay(config.angle_delay);
    distance_handler_->setNoises(config.distance_noise_meas);
    distance_handler_->setDelay(config.distance_delay);

    if ((level & msf_updates::SphericalPositionSensor_INIT_FILTER)
        && config.core_init_filter == true) {
      init(1.0);
      config.core_init_filter = false;
    }
  }

  void init(double scale) const {
    if (scale < 0.001) {
      ROS_WARN_STREAM("Init scale is " << scale << " correcting to 1.");
      scale = 1;
    }

    Eigen::Matrix<double, 3, 1> p, v, b_w, b_a, g, w_m, a_m, p_ip, p_vc;
    Eigen::Quaternion<double> q;
    msf_core::MSF_Core<EKFState_T>::ErrorStateCov P;

    // Init values.
    g << 0, 0, 9.81;	        /// Gravity.
    b_w << 0, 0, 0;		/// Bias gyroscopes.
    b_a << 0, 0, 0;		/// Bias accelerometer.

    v << 0, 0, 0;			/// Robot velocity (IMU centered).
    w_m << 0, 0, 0;		/// Initial angular velocity.
    a_m = g;			/// Initial acceleration.

    // Set the initial yaw alignment of body to world (the frame in which the
    // position sensor measures).
    double yawinit = config_.yaw_init / 180 * M_PI;
    Eigen::Quaterniond yawq(cos(yawinit / 2), 0, 0, sin(yawinit / 2));
    yawq.normalize();

    q = yawq;

    P.setZero();  // Error state covariance; if zero, a default initialization
                  //in msf_core is used

    msf_core::Vector2 angles = angle_handler_->getAngleMeasurement();
    msf_core::Vector1 distance = distance_handler_->getDistanceMeasurement();
    // Check that angles(0) is theta and angles(1) is phi...
    p_vc(0, 0) = distance(0) * sin(angles(0)) * cos(angles(1));
    p_vc(1, 0) = distance(0) * sin(angles(0)) * sin(angles(1));
    p_vc(2, 0) = distance(0) * cos(angles(0));

    ROS_INFO_STREAM(
        "initial measurement pos:[" << p_vc.transpose()<<"] orientation: "
        <<STREAMQUAT(q));

        // Check if we have already input from the measurement sensor.
if(    p_vc.norm() == 0)
    ROS_WARN_STREAM("No measurements received yet to initialize position - using [0 0 0]");

    ros::NodeHandle pnh("~");
    pnh.param("position_sensor/init/p_ip/x", p_ip[0], 0.0);
    pnh.param("position_sensor/init/p_ip/y", p_ip[1], 0.0);
    pnh.param("position_sensor/init/p_ip/z", p_ip[2], 0.0);

    // Calculate initial attitude and position based on sensor measurements.
    p = p_vc - q.toRotationMatrix() * p_ip;

    // Prepare init "measurement".
    boost::shared_ptr < msf_core::MSF_InitMeasurement<EKFState_T>
        > meas(new msf_core::MSF_InitMeasurement<EKFState_T>(true));

    meas->setStateInitValue < StateDefinition_T::p > (p);
    meas->setStateInitValue < StateDefinition_T::v > (v);
    meas->setStateInitValue < StateDefinition_T::q > (q);
    meas->setStateInitValue < StateDefinition_T::b_w > (b_w);
    meas->setStateInitValue < StateDefinition_T::b_a > (b_a);
    meas->setStateInitValue < StateDefinition_T::p_ip > (p_ip);

    setP(meas->get_P());  // Call my set P function.
    meas->get_w_m() = w_m;
    meas->get_a_m() = a_m;
    meas->time = ros::Time::now().toSec();

    // Call initialization in core.
    msf_core_->init(meas);
  }

  // Prior to this call, all states are initialized to zero/identity.
  virtual void resetState(EKFState_T& state) const {
    UNUSED(state);
  }
  virtual void initState(EKFState_T& state) const {
    UNUSED(state);
  }

  virtual void calculateQAuxiliaryStates(EKFState_T& state, double dt) const {
    const msf_core::Vector3 npipv = msf_core::Vector3::Constant(
        config_.noise_p_ip);
    // Compute the blockwise Q values and store them with the states,
    // these then get copied by the core to the correct places in Qd.
    state.getQBlock<StateDefinition_T::p_ip>() =
        (dt * npipv.cwiseProduct(npipv)).asDiagonal();
  }

  virtual void setP(
      Eigen::Matrix<double, EKFState_T::nErrorStatesAtCompileTime,
          EKFState_T::nErrorStatesAtCompileTime>& P) const {
    UNUSED(P);
    // Nothing, we only use the simulated cov for the core plus diagonal for the rest.
  }

  virtual void augmentCorrectionVector(
      Eigen::Matrix<double, EKFState_T::nErrorStatesAtCompileTime, 1>& correction) const {
    UNUSED(correction);
  }

  virtual void sanityCheckCorrection(
      EKFState_T& delaystate,
      const EKFState_T& buffstate,
      Eigen::Matrix<double, EKFState_T::nErrorStatesAtCompileTime, 1>& correction) const {
    UNUSED(delaystate);
    UNUSED(buffstate);
    UNUSED(correction);
  }
};
}
#endif /* SPHERICAL_POSITION_MEASUREMENTMANAGER_H */
