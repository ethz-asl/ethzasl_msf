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
#ifndef POSITION_MEASUREMENTMANAGER_H
#define POSITION_MEASUREMENTMANAGER_H

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <msf_core/msf_core.h>
#include <msf_core/msf_sensormanagerROS.h>
#include <msf_core/msf_IMUHandler_ROS.h>
#include "msf_statedef.hpp"
#include <msf_updates/position_sensor_handler/position_sensorhandler.h>
#include <msf_updates/position_sensor_handler/position_measurement.h>
#include <msf_updates/SinglePositionSensorConfig.h>

#include "sensor_fusion_comm/InitScale.h"
#include "sensor_fusion_comm/InitTransform.h"

#include "sensor_msgs/Imu.h"

namespace msf_position_sensor {

bool isIMUDataReceived_ = 1;

typedef msf_updates::SinglePositionSensorConfig Config_T;
typedef dynamic_reconfigure::Server<Config_T> ReconfigureServer;
typedef shared_ptr<ReconfigureServer> ReconfigureServerPtr;

class PositionSensorManager : public msf_core::MSF_SensorManagerROS<
    msf_updates::EKFState> {
  typedef PositionSensorHandler<
      msf_updates::position_measurement::PositionMeasurement,
      PositionSensorManager> PositionSensorHandler_T;
  friend class PositionSensorHandler<
      msf_updates::position_measurement::PositionMeasurement,
      PositionSensorManager> ;
 public:
  typedef msf_updates::EKFState EKFState_T;
  typedef EKFState_T::StateSequence_T StateSequence_T;
  typedef EKFState_T::StateDefinition_T StateDefinition_T;

  PositionSensorManager(
      ros::NodeHandle pnh = ros::NodeHandle("~/position_sensor")) {
    imu_handler_.reset(
        new msf_core::IMUHandler_ROS<msf_updates::EKFState>(*this, "msf_core",
                                                            "imu_handler"));

    position_handler_.reset(
        new PositionSensorHandler_T(*this, "", "position_sensor"));
    AddHandler(position_handler_);

    reconf_server_.reset(new ReconfigureServer(pnh));
    ReconfigureServer::CallbackType f = boost::bind(
        &PositionSensorManager::Config, this, _1, _2);
    reconf_server_->setCallback(f);

    init_scale_srv_ = pnh.advertiseService("initialize_msf_scale",
                                            &PositionSensorManager::InitScale,
                                            this);
    init_transform_srv_ = pnh.advertiseService("initialize_msf_transform",
                                               &PositionSensorManager::InitTransform,
                                               this);
    sub_imu_state_input_ = pnh.subscribe("/imu0", 1,
        &PositionSensorManager::IMUCallback, this);
  }
  virtual ~PositionSensorManager() {
  }

  virtual const Config_T& Getcfg() {
    return config_;
  }

 private:
  shared_ptr<msf_core::IMUHandler_ROS<msf_updates::EKFState> > imu_handler_;
  shared_ptr<PositionSensorHandler_T> position_handler_;

  Config_T config_;
  ReconfigureServerPtr reconf_server_;

  ros::ServiceServer init_scale_srv_;
  ros::ServiceServer init_transform_srv_;
  ros::Subscriber sub_imu_state_input_;

  msf_core::Vector3 linacc;

  void IMUCallback(const sensor_msgs::ImuConstPtr &msg)
  {
	  if(!isIMUDataReceived_)
	  {
		  linacc << msg->linear_acceleration.x, msg->linear_acceleration.y, msg
			  ->linear_acceleration.z;
		  isIMUDataReceived_ = 1;
	  }
  }

  /**
   * \brief Dynamic reconfigure callback.
   */
  virtual void Config(Config_T &config, uint32_t level) {
    config_ = config;
    position_handler_->SetNoises(config.position_noise_meas);
    position_handler_->SetDelay(config.position_delay);
    if ((level & msf_updates::SinglePositionSensor_INIT_FILTER)
        && config.core_init_filter == true) {
      Init(1.0);
      config.core_init_filter = false;
    }
  }

  bool InitScale(sensor_fusion_comm::InitScale::Request &req,
                 sensor_fusion_comm::InitScale::Response &res) {
    ROS_INFO("Initialize filter with scale %f", req.scale);
    Init(req.scale);
    res.result = "Initialized scale";
    return true;
  }

  bool InitTransform(sensor_fusion_comm::InitTransform::Request &req,
                     sensor_fusion_comm::InitTransform::Response &res) {
    /*ROS_INFO("Initialize filter with quaternion q_wv = [%f, %f, %f, %f], scale %f",
             req.transform_wv.rotation.w, req.transform_wv.rotation.x, req.transform_wv.rotation.y, req.transform_wv.rotation.z,
             req.scale);*/
    Eigen::Quaternion<double> q(req.transform_wv.rotation.w,
                                req.transform_wv.rotation.x,
                                req.transform_wv.rotation.y,
                                req.transform_wv.rotation.z);
    q.normalize();
    ROS_INFO_STREAM("Initialize filter with transform q = ["<<
                    STREAMQUAT(q)<<"], scale "<<req.scale);
    InitTransform(q, req.scale);
    return true;
  }

  void Init(double scale) const {
    double yawinit = config_.position_yaw_init / 180 * M_PI;
    Eigen::Quaterniond yawq(cos(yawinit / 2), 0, 0, sin(yawinit / 2));
    yawq.normalize();
    InitTransform(yawq, scale);
  }

  void InitTransform(const Eigen::Quaternion<double> &q,
                     double scale) const {
    if (scale < 0.001) {
      MSF_WARN_STREAM("init scale is "<<scale<<" correcting to 1");
      scale = 1;
    }

    Eigen::Matrix<double, 3, 1> p, v, b_w, b_a, g, w_m, a_m, p_ip, p_wp;
    Eigen::Quaterniond q_acc, q_res;
    msf_core::MSF_Core<EKFState_T>::ErrorStateCov P;

    // Init values.
    g << 0, 0, 9.81;  /// Gravity.
    b_w << 0, 0, 0;		/// Bias gyroscopes.
    b_a << 0, 0, 0;		/// Bias accelerometer.

    v << 0, 0, 0;			/// Robot velocity (IMU centered).
    w_m << 0, 0, 0;		/// Initial angular velocity.

    P.setZero();  // Error state covariance; if zero, a default initialization
                  // in msf_core is used

    p_wp = position_handler_->GetPositionMeasurement();

    // Calculate the orientation from the accelerometer of topic /imu0
	MSF_INFO_STREAM("Waiting for IMU data ....!");
	isIMUDataReceived_ = 0;
	while(!isIMUDataReceived_ && ros::ok())
	{
		ros::getGlobalCallbackQueue()->callAvailable(
			ros::WallDuration(0.03));
	}
	if(!isIMUDataReceived_) {
		MSF_WARN_STREAM("No IMU measurements received yet to initialize attitude - using [1 0 0 0]");
	}
	else {
		MSF_WARN_STREAM("Received IMU measurements [" << linacc << "]");
		// Normalize acc vector
		Eigen::Vector3d e_acc = linacc.normalized();

		// Align with ez_world
		// Gravity unity vector in world frame
		Eigen::Vector3d ez_world(0.0, 0.0, 1.0);
		// Axis angle from world frame to IMU frame
		Eigen::Vector3d axis = ez_world.cross(e_acc).normalized();
		// Quaternion representation
		Eigen::Quaternion<double> q_acc_temp(Eigen::AngleAxis<double>(-std::acos(ez_world.transpose() * e_acc), axis));
		q_acc = q_acc_temp.normalized();
    }

	q_res = (q*q_acc).normalized();		// Keep yawinit

    MSF_INFO_STREAM("initial measurement pos:[" << p_wp.transpose() << "] orientation: " << STREAMQUAT(q_res));

    // check if we have already input from the measurement sensor
    if (p_wp.norm() == 0)
      MSF_WARN_STREAM("No measurements received yet to initialize position - using [0 0 0]");

    // Fetch some parameters
    ros::NodeHandle pnh("~");
    pnh.param("position_sensor/init/p_ip/x", p_ip.x(), 0.0);
    pnh.param("position_sensor/init/p_ip/y", p_ip.y(), 0.0);
    pnh.param("position_sensor/init/p_ip/z", p_ip.z(), 0.0);

    // Calculate initial attitude and position based on sensor measurements.
    p = p_wp - q_res.toRotationMatrix() * p_ip;

    a_m = q_res.inverse() * g;		// Initial acceleration.

    // Prepare init "measurement"
    // True means that we will also set the initialsensor readings.
    shared_ptr < msf_core::MSF_InitMeasurement<EKFState_T>
        > meas(new msf_core::MSF_InitMeasurement<EKFState_T>(true));

    meas->SetStateInitValue < StateDefinition_T::p > (p);
    meas->SetStateInitValue < StateDefinition_T::v > (v);
    meas->SetStateInitValue < StateDefinition_T::q > (q_res);
    meas->SetStateInitValue < StateDefinition_T::b_w > (b_w);
    meas->SetStateInitValue < StateDefinition_T::b_a > (b_a);
    meas->SetStateInitValue < StateDefinition_T::p_ip > (p_ip);

    SetStateCovariance(meas->GetStateCovariance());  // Call my set P function.
    meas->Getw_m() = w_m;
    meas->Geta_m() = a_m;
    meas->time = ros::Time::now().toSec();

    // Call initialization in core.
    msf_core_->Init(meas);
  }

  // Prior to this call, all states are initialized to zero/identity.
  virtual void ResetState(EKFState_T& state) const {
    UNUSED(state);
  }
  virtual void InitState(EKFState_T& state) const {
    UNUSED(state);
  }

  virtual void CalculateQAuxiliaryStates(EKFState_T& state, double dt) const {
    const msf_core::Vector3 npipv = msf_core::Vector3::Constant(
        config_.position_noise_p_ip);

    // Compute the blockwise Q values and store them with the states,
    //these then get copied by the core to the correct places in Qd.
    state.GetQBlock<StateDefinition_T::p_ip>() =
        (dt * npipv.cwiseProduct(npipv)).asDiagonal();
  }

  virtual void SetStateCovariance(
      Eigen::Matrix<double, EKFState_T::nErrorStatesAtCompileTime,
          EKFState_T::nErrorStatesAtCompileTime>& P) const {
    UNUSED(P);
    // Nothing, we only use the simulated cov for the core plus diagonal for the
    // rest.
  }

  virtual void AugmentCorrectionVector(
      Eigen::Matrix<double, EKFState_T::nErrorStatesAtCompileTime, 1>& correction) const {
    UNUSED(correction);
  }

  virtual void SanityCheckCorrection(
      EKFState_T& delaystate,
      const EKFState_T& buffstate,
      Eigen::Matrix<double, EKFState_T::nErrorStatesAtCompileTime, 1>& correction) const {
    UNUSED(delaystate);
    UNUSED(buffstate);
    UNUSED(correction);
  }
};
}
#endif  // POSITION_MEASUREMENTMANAGER_H
