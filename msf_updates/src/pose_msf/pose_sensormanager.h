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
#include <msf_updates/SinglePoseSensorConfig.h>

namespace msf_pose_sensor{

typedef msf_updates::SinglePoseSensorConfig Config_T;
typedef dynamic_reconfigure::Server<Config_T> ReconfigureServer;
typedef boost::shared_ptr<ReconfigureServer> ReconfigureServerPtr;

class PoseSensorManager : public msf_core::MSF_SensorManagerROS<msf_updates::EKFState>
{
  friend class PoseSensorHandler;
public:
  typedef msf_updates::EKFState EKFState_T;
  typedef EKFState_T::StateSequence_T StateSequence_T;
  typedef EKFState_T::StateDefinition_T StateDefinition_T;

  PoseSensorManager(ros::NodeHandle pnh = ros::NodeHandle("~/pose_sensor"))
  {
    bool poseabsolute; ///<does the pose sensor provides absolute measurements

    pnh.param("absolute_measurements", poseabsolute, true);

    pose_handler_.reset(new PoseSensorHandler(*this, poseabsolute));
    addHandler(pose_handler_);

    reconf_server_.reset(new ReconfigureServer(pnh));
    ReconfigureServer::CallbackType f = boost::bind(&PoseSensorManager::config, this, _1, _2);
    reconf_server_->setCallback(f);
  }
  virtual ~PoseSensorManager(){}

private:
  boost::shared_ptr<PoseSensorHandler> pose_handler_;

  Config_T config_;
  ReconfigureServerPtr reconf_server_; ///< dynamic reconfigure server

  /**
   * \brief dynamic reconfigure callback
   */
  virtual void config(Config_T &config, uint32_t level){
    config_ = config;
    pose_handler_->setNoises(config.noise_position, config.noise_attitude);
    pose_handler_->setDelay(config.delay);
    if((level & msf_updates::SinglePoseSensor_INIT_FILTER) && config.init_filter == true){
      init(config.initial_scale);
      config.init_filter = false;
    }
    //Init call with "set height" checkbox
    if((level & msf_updates::SinglePoseSensor_SET_HEIGHT) && config.set_height == true){
      Eigen::Matrix<double, 3, 1> p = pose_handler_->getPositionMeasurement();
      if (p.norm() == 0){
           ROS_WARN_STREAM("No measurements received yet to initialize position. Height init not allowed.");
           return;
      }
      double scale =  p[2]/config.height;
      init(scale);
      config.set_height = false;
    }
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
    p_vw.setZero(); //vision-world position drift

    P.setZero(); // error state covariance; if zero, a default initialization in msf_core is used

    p_vc = pose_handler_->getPositionMeasurement();
    q_cv = pose_handler_->getAttitudeMeasurement();

    ROS_INFO_STREAM("initial measurement pos:["<<p_vc.transpose()<<"] orientation:["<<q_cv.w()<<", "<<q_cv.x()<<", "<<q_cv.y()<<", "<<q_cv.z()<<"]");

    // check if we have already input from the measurement sensor
    if (p_vc.norm() == 0)
      ROS_WARN_STREAM("No measurements received yet to initialize position - using [0 0 0]");
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

    if (this->config_.fixed_scale)
    {
      typedef typename msf_tmp::getEnumStateType<StateSequence_T, StateDefinition_T::L>::value L_type;
      const int L_indexInErrorState = msf_tmp::getStateIndexInErrorState<StateSequence_T, StateDefinition_T::L>::value;

      for(int i = 0 ; i < msf_tmp::StripConstReference<L_type>::result_t::sizeInCorrection_ ; ++i){
        correction_(L_indexInErrorState + i) = 0; //scale
      }
    }

    if (this->config_.fixed_calib)
    {
      typedef typename msf_tmp::getEnumStateType<StateSequence_T, StateDefinition_T::q_ci>::value q_ci_type;
      const int q_ci_indexInErrorState = msf_tmp::getStateIndexInErrorState<StateSequence_T, StateDefinition_T::q_ci>::value;

      for(int i = 0 ; i < msf_tmp::StripConstReference<q_ci_type>::result_t::sizeInCorrection_ ; ++i){
        correction_(q_ci_indexInErrorState + i) = 0; //q_ic roll, pitch, yaw
      }

      typedef typename msf_tmp::getEnumStateType<StateSequence_T, StateDefinition_T::p_ci>::value p_ci_type;
      const int p_ci_indexInErrorState = msf_tmp::getStateIndexInErrorState<StateSequence_T, StateDefinition_T::p_ci>::value;

      for(int i = 0 ; i < msf_tmp::StripConstReference<p_ci_type>::result_t::sizeInCorrection_ ; ++i){
        correction_(p_ci_indexInErrorState + i) = 0; //p_ci x,y,z
      }
    }
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
