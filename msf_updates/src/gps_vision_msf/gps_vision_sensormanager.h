/*

Copyright (c) 2010, Stephan Weiss, ASL, ETH Zurich, Switzerland
You can contact the author at <stephan dot weiss at ieee dot org>
Copyright (c) 2012, Simon Lynen, ASL, ETH Zurich, Switzerland
You can contact the author at <slynen at ethz dot ch>
 Copyright (c) 2012, Markus Achtelik, ASL, ETH Zurich, Switzerland
 You can contact the author at <acmarkus at ethz dot ch>

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

#ifndef POSE_MEASUREMENTMANAGER_H
#define POSE_MEASUREMENTMANAGER_H

#include <ros/ros.h>

#include <msf_core/msf_core.h>
#include <msf_core/msf_sensormanagerROS.h>
#include "msf_statedef.hpp"
#include <msf_updates/pose_sensor_handler/pose_sensorhandler.h>
#include <msf_updates/pose_sensor_handler/pose_measurement.h>
#include <msf_updates/position_sensor_handler/position_sensorhandler.h>
#include <msf_updates/position_sensor_handler/position_measurement.h>
#include <msf_updates/GPSVisionSensorConfig.h>

namespace msf_updates{

typedef msf_updates::GPSVisionSensorConfig Config_T;
typedef dynamic_reconfigure::Server<Config_T> ReconfigureServer;
typedef boost::shared_ptr<ReconfigureServer> ReconfigureServerPtr;

class GPSVisionSensorManager : public msf_core::MSF_SensorManagerROS<msf_updates::EKFState>
{
  typedef msf_pose_sensor::PoseSensorHandler<msf_updates::pose_measurement::PoseMeasurement, GPSVisionSensorManager> PoseSensorHandler_T;
  friend class msf_pose_sensor::PoseSensorHandler<msf_updates::pose_measurement::PoseMeasurement, GPSVisionSensorManager>;
  typedef msf_position_sensor::PositionSensorHandler<msf_updates::position_measurement::PositionMeasurement, GPSVisionSensorManager> PositionSensorHandler_T;
  friend class msf_position_sensor::PositionSensorHandler<msf_updates::position_measurement::PositionMeasurement, GPSVisionSensorManager>;
public:
  typedef msf_updates::EKFState EKFState_T;
  typedef EKFState_T::StateSequence_T StateSequence_T;
  typedef EKFState_T::StateDefinition_T StateDefinition_T;

  GPSVisionSensorManager(ros::NodeHandle pnh = ros::NodeHandle("~"))
  {
    bool poseabsolute = true; ///<does the pose sensor provides absolute measurements
    bool distortmeas = false; ///<distort the pose measurements TODO make param

    pnh.param("pose_sensor/absolute_pose_measurements", poseabsolute, true);

    if(poseabsolute){
      ROS_ERROR_STREAM("You defined that your pose sensor has global updates, but at the same time you are using GPS.");
      poseabsolute = false;
    }

    pose_handler_.reset(new PoseSensorHandler_T(*this, poseabsolute, distortmeas));
    addHandler(pose_handler_);

    bool positionabsolute = true;
    pnh.param("position_sensor/absolute_position_measurements", positionabsolute, true);

    position_handler_.reset(new PositionSensorHandler_T(*this, positionabsolute));
    addHandler(position_handler_);

    reconf_server_.reset(new ReconfigureServer(pnh));
    ReconfigureServer::CallbackType f = boost::bind(&GPSVisionSensorManager::config, this, _1, _2);
    reconf_server_->setCallback(f);
  }
  virtual ~GPSVisionSensorManager(){}

  virtual const Config_T& getcfg(){
    return config_;
  }

private:
  boost::shared_ptr<PoseSensorHandler_T> pose_handler_;
  boost::shared_ptr<PositionSensorHandler_T> position_handler_;

  Config_T config_;
  ReconfigureServerPtr reconf_server_; ///< dynamic reconfigure server

  /**
   * \brief dynamic reconfigure callback
   */
  virtual void config(Config_T &config, uint32_t level){
    config_ = config;
    pose_handler_->setNoises(config.vision_noise_position, config.vision_noise_attitude);
    pose_handler_->setDelay(config.delay);
    position_handler_->setNoises(config.gps_noise_position);
    if((level & msf_updates::GPSVisionSensor_INIT_FILTER) && config.init_filter == true){
      init(config.initial_scale);
      config.init_filter = false;
    }
    //Init call with "set height" checkbox
    if((level & msf_updates::GPSVisionSensor_SET_HEIGHT) && config.set_height == true){
      Eigen::Matrix<double, 3, 1> p = pose_handler_->getPositionMeasurement();
      if (p.norm() == 0){
           ROS_WARN_STREAM("No measurements received yet to initialize position. Height init not allowed.");
           return;
      }
      double scale =  p[2]/config.height;
      init(scale);
      config.set_height = false;
    }
    ROS_INFO_STREAM("fixed_bias: "<<config_.fixed_bias);
    ROS_INFO_STREAM("fixed_scale: "<<config_.fixed_scale);
    ROS_INFO_STREAM("fixed_pos_drift_vw: "<<config_.fixed_pos_drift_vw);
    ROS_INFO_STREAM("fixed_att_drift_vw: "<<config_.fixed_att_drift_vw);
    ROS_INFO_STREAM("fixed_calib_att: "<<config_.fixed_calib_att);
    ROS_INFO_STREAM("fixed_calib_pos: "<<config_.fixed_calib_pos);
  }

  void init(double scale)
  {
    Eigen::Matrix<double, 3, 1> p, v, b_w, b_a, g, w_m, a_m, p_ci, p_vc, p_vw, p_gps;
    Eigen::Quaternion<double> q, q_wv, q_ci, q_cv;
    msf_core::MSF_Core<EKFState_T>::ErrorStateCov P;

    // init values
    g << 0, 0, 9.81;	        /// gravity
    b_w << 0,0,0;		/// bias gyroscopes
    b_a << 0,0,0;		/// bias accelerometer

    v << 0,0,0;			/// robot velocity (IMU centered)
    w_m << 0,0,0;		/// initial angular velocity
    a_m = g;			/// initial acceleration

    q_wv.setIdentity(); // vision-world rotation drift
    p_vw.setZero(); //vision-world position drift

    P.setZero(); // error state covariance; if zero, a default initialization in msf_core is used

    p_gps = position_handler_->getPositionMeasurement();

    p_vc = pose_handler_->getPositionMeasurement();
    q_cv = pose_handler_->getAttitudeMeasurement();

    ROS_INFO_STREAM("initial measurement vision: pos:["<<p_vc.transpose()<<"] orientation:["<<q_cv.w()<<", "<<q_cv.x()<<", "<<q_cv.y()<<", "<<q_cv.z()<<"]");
    ROS_INFO_STREAM("initial measurement gps: pos:["<<p_gps.transpose()<<"]");

    // check if we have already input from the measurement sensor
    if (p_vc.norm() == 0)
      ROS_WARN_STREAM("No measurements received yet to initialize vision position - using [0 0 0]");
    if (p_gps.norm() == 0)
      ROS_WARN_STREAM("No measurements received yet to initialize gps position - using [0 0 0]");
    if ((p_vc.norm() == 1) & (q_cv.w() == 1))
      ROS_WARN_STREAM("No measurements received yet to initialize attitude - using [1 0 0 0]");

    ros::NodeHandle pnh("~");
    pnh.param("init/p_ci/x", p_ci[0], 0.0);
    pnh.param("init/p_ci/y", p_ci[1], 0.0);
    pnh.param("init/p_ci/z", p_ci[2], 0.0);

    pnh.param("init/q_ci/w", q_ci.w(), 1.0);
    pnh.param("init/q_ci/x", q_ci.x(), 0.0);
    pnh.param("init/q_ci/y", q_ci.y(), 0.0);
    pnh.param("init/q_ci/z", q_ci.z(), 0.0);
    q_ci.normalize();


    // calculate initial attitude and position based on sensor measurements
    q = (q_ci * q_cv.conjugate() * q_wv).conjugate();
    q.normalize();

    Eigen::Matrix<double, 3, 1> p_vision = q_wv.conjugate().toRotationMatrix() * p_vc / scale - q.toRotationMatrix() * p_ci;

    p = p_gps;
    p_vw = p_vision - p;

    //prepare init "measurement"
    boost::shared_ptr<msf_core::MSF_InitMeasurement<EKFState_T> > meas(new msf_core::MSF_InitMeasurement<EKFState_T>(true)); //hand over that we will also set the sensor readings

    meas->setStateInitValue<StateDefinition_T::p>(p);
    meas->setStateInitValue<StateDefinition_T::v>(v);
    meas->setStateInitValue<StateDefinition_T::q>(q);
    meas->setStateInitValue<StateDefinition_T::b_w>(b_w);
    meas->setStateInitValue<StateDefinition_T::b_a>(b_a);
    meas->setStateInitValue<StateDefinition_T::L>(Eigen::Matrix<double, 1, 1>::Constant(scale));
    meas->setStateInitValue<StateDefinition_T::q_wv>(q_wv);
    meas->setStateInitValue<StateDefinition_T::p_vw>(p_vw);
    meas->setStateInitValue<StateDefinition_T::q_ci>(q_ci);
    meas->setStateInitValue<StateDefinition_T::p_ci>(p_ci);

    setP(meas->get_P()); //call my set P function
    meas->get_w_m() = w_m;
    meas->get_a_m() = a_m;
    meas->time = ros::Time::now().toSec();

    // call initialization in core
    msf_core_->init(meas);
  }

  //prior to this call, all states are initialized to zero/identity
  virtual void resetState(EKFState_T& state){
    //set scale to 1

    Eigen::Matrix<double, 1, 1> scale;
    scale << 1.0;
    state.set<StateDefinition_T::L>(scale);
  }
  virtual void initState(EKFState_T& state){


  }

  virtual void calculateQAuxiliaryStates(EKFState_T& state, double dt){
    const msf_core::Vector3 nqwvv = msf_core::Vector3::Constant(config_.noise_qwv);
    const msf_core::Vector3 npwvv = msf_core::Vector3::Constant(config_.noise_pvw);
    const msf_core::Vector3 nqciv = msf_core::Vector3::Constant(config_.noise_qci);
    const msf_core::Vector3 npicv = msf_core::Vector3::Constant(config_.noise_pci);
    const msf_core::Vector1 n_L = msf_core::Vector1::Constant(config_.noise_scale);

    //compute the blockwise Q values and store them with the states,
    //these then get copied by the core to the correct places in Qd
    state.getQBlock<StateDefinition_T::L>() 	= (dt * n_L.cwiseProduct(n_L)).asDiagonal();
    state.getQBlock<StateDefinition_T::q_wv>() = (dt * nqwvv.cwiseProduct(nqwvv)).asDiagonal();
    state.getQBlock<StateDefinition_T::p_vw>() = (dt * npwvv.cwiseProduct(npwvv)).asDiagonal();
    state.getQBlock<StateDefinition_T::q_ci>() = (dt * nqciv.cwiseProduct(nqciv)).asDiagonal();
    state.getQBlock<StateDefinition_T::p_ci>() = (dt * npicv.cwiseProduct(npicv)).asDiagonal();
  }

  virtual void setP(Eigen::Matrix<double, EKFState_T::nErrorStatesAtCompileTime, EKFState_T::nErrorStatesAtCompileTime>& P){

    //nothing, we only use the simulated cov for the core plus diagonal for the rest

  }

  virtual void augmentCorrectionVector(Eigen::Matrix<double, EKFState_T::nErrorStatesAtCompileTime,1>& correction_){

  }

  virtual void sanityCheckCorrection(EKFState_T& delaystate, const EKFState_T& buffstate,
                                     Eigen::Matrix<double, EKFState_T::nErrorStatesAtCompileTime,1>& correction_){

    const EKFState_T& state = delaystate;
    if (state.get<StateDefinition_T::L>()(0) < 0)
    {
      ROS_WARN_STREAM_THROTTLE(1,"Negative scale detected: " << state.get<StateDefinition_T::L>()(0) << ". Correcting to 0.1");
      Eigen::Matrix<double, 1, 1> L_;
      L_ << 0.1;
      delaystate.set<StateDefinition_T::L>(L_);
    }

  }

};

}
#endif /* POSE_MEASUREMENTMANAGER_H */
