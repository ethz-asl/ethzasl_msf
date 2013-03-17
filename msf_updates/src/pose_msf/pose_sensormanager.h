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
#include <msf_updates/SinglePoseSensorConfig.h>

namespace msf_pose_sensor{

typedef msf_updates::SinglePoseSensorConfig Config_T;
typedef dynamic_reconfigure::Server<Config_T> ReconfigureServer;
typedef boost::shared_ptr<ReconfigureServer> ReconfigureServerPtr;

class PoseSensorManager : public msf_core::MSF_SensorManagerROS<msf_updates::EKFState>
{
  typedef PoseSensorHandler<msf_updates::pose_measurement::PoseMeasurement, PoseSensorManager> PoseSensorHandler_T;
  friend class PoseSensorHandler<msf_updates::pose_measurement::PoseMeasurement, PoseSensorManager>;
public:
  typedef msf_updates::EKFState EKFState_T;
  typedef EKFState_T::StateSequence_T StateSequence_T;
  typedef EKFState_T::StateDefinition_T StateDefinition_T;

  PoseSensorManager(ros::NodeHandle pnh = ros::NodeHandle("~/pose_sensor"))
  {
    bool distortmeas = false; ///<distort the pose measurements TODO make param

    pose_handler_.reset(new PoseSensorHandler_T(*this, "", "pose_sensor", distortmeas));


    addHandler(pose_handler_);

    reconf_server_.reset(new ReconfigureServer(pnh));
    ReconfigureServer::CallbackType f = boost::bind(&PoseSensorManager::config, this, _1, _2);
    reconf_server_->setCallback(f);
  }
  virtual ~PoseSensorManager(){}

  virtual const Config_T& getcfg(){
    return config_;
  }

private:
  boost::shared_ptr<PoseSensorHandler_T> pose_handler_;

  Config_T config_;
  ReconfigureServerPtr reconf_server_; ///< dynamic reconfigure server

  /**
   * \brief dynamic reconfigure callback
   */
  virtual void config(Config_T &config, uint32_t level){
    config_ = config;
    pose_handler_->setNoises(config.pose_noise_meas_p, config.pose_noise_meas_q);
    pose_handler_->setDelay(config.pose_delay);
    if((level & msf_updates::SinglePoseSensor_INIT_FILTER) && config.core_init_filter == true){
      init(config.pose_initial_scale);
      config.core_init_filter = false;
    }
    //Init call with "set height" checkbox
    if((level & msf_updates::SinglePoseSensor_SET_HEIGHT) && config.core_set_height == true){
      Eigen::Matrix<double, 3, 1> p = pose_handler_->getPositionMeasurement();
      if (p.norm() == 0){
           ROS_WARN_STREAM("No measurements received yet to initialize position. Height init not allowed.");
           return;
      }
      double scale =  p[2]/config.core_height;
      init(scale);
      config.core_set_height = false;
    }
    ROS_INFO_STREAM("pose_fixed_scale: "<<config_.pose_fixed_scale);
    ROS_INFO_STREAM("pose_fixed_p_vw: "<<config_.pose_fixed_p_vw);
    ROS_INFO_STREAM("pose_fixed_q_vw: "<<config_.pose_fixed_q_wv);
    ROS_INFO_STREAM("pose_fixed_q_ci: "<<config_.pose_fixed_q_ci);
    ROS_INFO_STREAM("pose_fixed_p_ci: "<<config_.pose_fixed_p_ci);
  }

  void init(double scale)
  {
    Eigen::Matrix<double, 3, 1> p, v, b_w, b_a, g, w_m, a_m, p_ci, p_vc, p_vw;
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
    q_wv = Eigen::Quaterniond(cos(90. / 180. * M_PI / 2.),0 ,0 , sin(90. / 180. * M_PI / 2.));
    q_wv.normalize();
    p_vw.setZero(); //vision-world position drift

    P.setZero(); // error state covariance; if zero, a default initialization in msf_core is used

    p_vc = pose_handler_->getPositionMeasurement();
    q_cv = pose_handler_->getAttitudeMeasurement();

    ROS_INFO_STREAM("initial measurement pos:["<<p_vc.transpose()<<"] orientation:["<<q_cv.w()<<", "<<q_cv.x()<<", "<<q_cv.y()<<", "<<q_cv.z()<<"]");

    // check if we have already input from the measurement sensor
    if (p_vc.norm() == 0)
      ROS_WARN_STREAM("No measurements received yet to initialize position - using [0 0 0]");
    if (q_cv.w() == 1)
      ROS_WARN_STREAM("No measurements received yet to initialize attitude - using [1 0 0 0]");

    ros::NodeHandle pnh("~");
    pnh.param("pose_sensor/init/p_ci/x", p_ci[0], 0.0);
    pnh.param("pose_sensor/init/p_ci/y", p_ci[1], 0.0);
    pnh.param("pose_sensor/init/p_ci/z", p_ci[2], 0.0);

    pnh.param("pose_sensor/init/q_ci/w", q_ci.w(), 1.0);
    pnh.param("pose_sensor/init/q_ci/x", q_ci.x(), 0.0);
    pnh.param("pose_sensor/init/q_ci/y", q_ci.y(), 0.0);
    pnh.param("pose_sensor/init/q_ci/z", q_ci.z(), 0.0);
    q_ci.normalize();


    // calculate initial attitude and position based on sensor measurements
    if (q_cv.w() == 1){ //if there is no pose measurement, only apply q_vw
      q = q_wv;
    }else{ //if there is a pose measurement, apply q_ci and q_vw to get initial attitude
      q = (q_ci * q_cv.conjugate() * q_wv).conjugate();
    }

    q.normalize();
    p = q_wv.conjugate().toRotationMatrix() * p_vc / scale - q.toRotationMatrix() * p_ci;

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
    const msf_core::Vector3 nqwvv = msf_core::Vector3::Constant(config_.pose_noise_qwv);
    const msf_core::Vector3 npwvv = msf_core::Vector3::Constant(config_.pose_noise_pvw);
    const msf_core::Vector3 nqciv = msf_core::Vector3::Constant(config_.pose_noise_qci);
    const msf_core::Vector3 npicv = msf_core::Vector3::Constant(config_.pose_noise_pic);
    const msf_core::Vector1 n_L = msf_core::Vector1::Constant(config_.pose_noise_scale);

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
