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

#ifndef POSE_PRESSURE_MEASUREMENTMANAGER_H
#define POSE_PRESSURE_MEASUREMENTMANAGER_H

#include <ros/ros.h>
#include <msf_core/msf_sensormanagerROS.h>
#include <msf_core/msf_core.h>
#include "msf_statedef.hpp"
#include <msf_updates/pose_sensor_handler/pose_sensorhandler.h>
#include <msf_updates/pressure_sensor_handler/pressure_sensorhandler.h>
#include <msf_updates/pose_sensor_handler/pose_measurement.h>
#include <msf_updates/PosePressureSensorConfig.h>
#include <msf_updates/SinglePoseSensorConfig.h>

namespace msf_pose_pressure_sensor{

typedef msf_updates::PosePressureSensorConfig Config_T;
typedef dynamic_reconfigure::Server<Config_T> ReconfigureServer;
typedef boost::shared_ptr<ReconfigureServer> ReconfigureServerPtr;

class PosePressureSensorManager : public msf_core::MSF_SensorManagerROS<msf_updates::EKFState>
{
  typedef msf_pose_sensor::PoseSensorHandler<msf_updates::pose_measurement::PoseMeasurement, PosePressureSensorManager> PoseSensorHandler_T;
  friend class msf_pose_sensor::PoseSensorHandler<msf_updates::pose_measurement::PoseMeasurement, PosePressureSensorManager>;
public:
  typedef msf_updates::EKFState EKFState_T;
  typedef EKFState_T::StateSequence_T StateSequence_T;
  typedef EKFState_T::StateDefinition_T StateDefinition_T;

  PosePressureSensorManager(ros::NodeHandle pnh = ros::NodeHandle("~/pose_pressure_sensor"))
  {
    bool distortmeasurement = true;
    pose_handler_.reset(new PoseSensorHandler_T(*this, "", "pose_sensor", distortmeasurement));
    addHandler(pose_handler_);

    pressure_handler_.reset(new msf_pressure_sensor::PressureSensorHandler(*this, "", "pressure_sensor"));
    addHandler(pressure_handler_);

    reconf_server_.reset(new ReconfigureServer(pnh));
    ReconfigureServer::CallbackType f = boost::bind(&PosePressureSensorManager::config, this, _1, _2);
    reconf_server_->setCallback(f);
  }

  virtual ~PosePressureSensorManager(){}

  virtual const Config_T& getcfg(){
    return config_;
  }

private:
  boost::shared_ptr<PoseSensorHandler_T> pose_handler_;
  boost::shared_ptr<msf_pressure_sensor::PressureSensorHandler> pressure_handler_;

  Config_T config_;
  ReconfigureServerPtr reconf_server_; ///< dynamic reconfigure server

  /**
   * \brief dynamic reconfigure callback
   */
  virtual void config(Config_T &config, uint32_t level){
    config_ = config;
    //Init call with "init filter" checkbox
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

    //pass the noise values to the sensor handlers
    pose_handler_->setNoises(config.pose_noise_meas_p, config.pose_noise_meas_q);
    pressure_handler_->setNoises(config.press_noise_meas_p);

  }

  void init(double scale)
  {

    Eigen::Matrix<double, 3, 1> p, v, b_w, b_a, g, w_m, a_m, p_ic, p_vc;
    Eigen::Quaternion<double> q, q_wv, q_ic, q_vc;
    Eigen::Matrix<double, 1, 1> b_p;
    msf_core::MSF_Core<EKFState_T>::ErrorStateCov P;

    // init values
    g << 0, 0, 9.81;	        /// gravity
    b_w << 0,0,0;		/// bias gyroscopes
    b_a << 0,0,0;		/// bias accelerometer

    v << 0,0,0;			/// robot velocity (IMU centered)
    w_m << 0,0,0;		/// initial angular velocity
    a_m =g;			/// initial acceleration

    q_wv.setIdentity(); // vision-world rotation drift

    P.setZero(); // error state covariance; if zero, a default initialization in msf_core is used

    p_vc = pose_handler_->getPositionMeasurement();
    q_vc = pose_handler_->getAttitudeMeasurement();

    b_p << pose_handler_->getPositionMeasurement()(2) / scale - pressure_handler_->getPressureMeasurement()(0); /// pressure drift state

    // check if we have already input from the measurement sensor
    if (p_vc.norm() == 0)
      ROS_WARN_STREAM("No measurements received yet to initialize position - using [0 0 0]");
    if (q_vc.w() == 1)
      ROS_WARN_STREAM("No measurements received yet to initialize attitude - using [1 0 0 0]");

    ros::NodeHandle pnh("~");
    pnh.param("pose_sensor/init/p_ic/x", p_ic[0], 0.0);
    pnh.param("pose_sensor/init/p_ic/y", p_ic[1], 0.0);
    pnh.param("pose_sensor/init/p_ic/z", p_ic[2], 0.0);

    pnh.param("pose_sensor/init/q_ic/w", q_ic.w(), 1.0);
    pnh.param("pose_sensor/init/q_ic/x", q_ic.x(), 0.0);
    pnh.param("pose_sensor/init/q_ic/y", q_ic.y(), 0.0);
    pnh.param("pose_sensor/init/q_ic/z", q_ic.z(), 0.0);
    q_ic.normalize();


    // calculate initial attitude and position based on sensor measurements
    if (q_vc.w() == 1){ //if there is no pose measurement, only apply q_wv
      q = q_wv;
    }else{ //if there is a pose measurement, apply q_ic and q_wv to get initial attitude
      q = (q_ic * q_vc.conjugate() * q_wv).conjugate();
    }
    q.normalize();
    p = q_wv.conjugate().toRotationMatrix() * p_vc / scale - q.toRotationMatrix() * p_ic;

    //prepare init "measurement"
    boost::shared_ptr<msf_core::MSF_InitMeasurement<EKFState_T> > meas(new msf_core::MSF_InitMeasurement<EKFState_T>(true)); //hand over that we will also set the sensor readings

    meas->setStateInitValue<StateDefinition_T::p>(p);
    meas->setStateInitValue<StateDefinition_T::v>(v);
    meas->setStateInitValue<StateDefinition_T::q>(q);
    meas->setStateInitValue<StateDefinition_T::b_w>(b_w);
    meas->setStateInitValue<StateDefinition_T::b_a>(b_a);
    meas->setStateInitValue<StateDefinition_T::L>(Eigen::Matrix<double, 1, 1>::Constant(scale));
    meas->setStateInitValue<StateDefinition_T::q_wv>(q_wv);
    meas->setStateInitValue<StateDefinition_T::q_ic>(q_ic);
    meas->setStateInitValue<StateDefinition_T::p_ic>(p_ic);
    meas->setStateInitValue<StateDefinition_T::b_p>(b_p);

    setP(meas->get_P()); //call my set P function
    meas->get_w_m() = w_m;
    meas->get_a_m() = a_m;
    meas->time = ros::Time::now().toSec();

    // call initialization in core
    msf_core_->init(meas);

    ROS_INFO_STREAM("filter initialized to: \n" <<
                    "position: [" << p[0] << ", " << p[1] << ", " << p[2] << "]" << std::endl <<
                    "scale:" << scale << std::endl <<
                    "attitude (w,x,y,z): [" << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << std::endl <<
                    "p_ic: [" << p_ic[0] << ", " << p_ic[1] << ", " << p_ic[2] << std::endl <<
                    "q_ic: (w,x,y,z): [" << q_ic.w() << ", " << q_ic.x() << ", " << q_ic.y() << ", " << q_ic.z() << "]");
  }

  //prior to this call, all states are initialized to zero/identity
  virtual void resetState(EKFState_T& state){
    //set scale to 1

    Eigen::Matrix<double, 1, 1> scale;
    scale << 1.0;
    state.set<StateDefinition_T::L>(scale);
  }
  virtual void initState(EKFState_T& state){
    UNUSED(state);
  }

  virtual void calculateQAuxiliaryStates(EKFState_T& state, double dt){
    const msf_core::Vector3 nqwvv = msf_core::Vector3::Constant(config_.pose_noise_q_wv);
    const msf_core::Vector3 nqicv = msf_core::Vector3::Constant(config_.pose_noise_q_ic);
    const msf_core::Vector3 npicv = msf_core::Vector3::Constant(config_.pose_noise_p_ic);
    const msf_core::Vector1 n_L = msf_core::Vector1::Constant(config_.pose_noise_scale);
    const msf_core::Vector1 nb_p = msf_core::Vector1::Constant(config_.press_noise_bias_p);

    //compute the blockwise Q values and store them with the states,
    //these then get copied by the core to the correct places in Qd
    state.getQBlock<StateDefinition_T::L>() 	= (dt * n_L.cwiseProduct(n_L)).asDiagonal();
    state.getQBlock<StateDefinition_T::q_wv>() = (dt * nqwvv.cwiseProduct(nqwvv)).asDiagonal();
    state.getQBlock<StateDefinition_T::q_ic>() = (dt * nqicv.cwiseProduct(nqicv)).asDiagonal();
    state.getQBlock<StateDefinition_T::p_ic>() = (dt * npicv.cwiseProduct(npicv)).asDiagonal();
    state.getQBlock<StateDefinition_T::b_p>() = (dt * nb_p.cwiseProduct(nb_p)).asDiagonal();
  }

  virtual void setP(Eigen::Matrix<double, EKFState_T::nErrorStatesAtCompileTime, EKFState_T::nErrorStatesAtCompileTime>& P){
    UNUSED(P);
  }

  virtual void augmentCorrectionVector(Eigen::Matrix<double, EKFState_T::nErrorStatesAtCompileTime,1>& correction){
    UNUSED(correction);
  }

  virtual void sanityCheckCorrection(EKFState_T& delaystate, const EKFState_T& buffstate,
                                     Eigen::Matrix<double, EKFState_T::nErrorStatesAtCompileTime,1>& correction){

    UNUSED(buffstate);
    UNUSED(correction);

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
