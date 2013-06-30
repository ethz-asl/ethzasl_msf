/*

 Copyright (c) 2012, Simon Lynen, ASL, ETH Zurich, Switzerland
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

#ifndef SENSORMANAGER_H
#define SENSORMANAGER_H

#include <Eigen/Dense>
#include <string.h>
#include <msf_core/msf_types.tpp>
#include <msf_core/msf_statevisitor.h>
#include <msf_core/msf_macros.h>

namespace msf_core {

template<typename EKFState_T>
class SensorHandler;

template<typename EKFState_T>
class MSF_Core;

/** \class MSF_SensorManager
 * \brief A manager for a given sensor set. Handlers for individual sensors (camera/vicon etc.) are
 * registered with this class as handlers of particular sensors. This class also owns the
 * EKF core instance and handles the initialization of the filter
 */
template<typename EKFState_T>
class MSF_SensorManager : public StateVisitor<EKFState_T> {
 private:
  int sensorID_;
 protected:
  typedef std::vector<shared_ptr<SensorHandler<EKFState_T> > > Handlers;
  Handlers handlers;  ///<a list of sensor handlers which provide measurements

  /**
   * used to determine if internal states get overwritten by the external
   * state prediction (online) or internal state prediction is performed
   * for log replay, when the external prediction is not available or should be
   * done on the host.
   */
  bool data_playback_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ;

  shared_ptr<MSF_Core<EKFState_T> > msf_core_;  ///< the ekf core instance

  MSF_SensorManager();

  bool data_playback(){return data_playback_;}

  virtual ~MSF_SensorManager() {

  }
  ;
  /***
   * add a new sensor handler to the list of handlers owned by this manager
   * a sensor handler is in turn owning the sensor (camera/vicon etc.)
   */
  void addHandler(shared_ptr<SensorHandler<EKFState_T> > handler) {
    handler->setSensorID(sensorID_++);
    handlers.push_back(handler);
  }

  /***
   * init function for the EKF
   */
  virtual void init(double scale) const = 0;

  /***
   * this method will be called for the user to set the initial state
   */
  virtual void initState(EKFState_T& state) const = 0;

  /***
   * this method will be called for the user to set the Q block entries for Auxiliary states
   * only changes to blocks in Q belonging to the auxiliary states are allowed / evaluated
   */
  virtual void calculateQAuxiliaryStates(EKFState_T& UNUSEDPARAM(state),
                                         double UNUSEDPARAM(dt)) const {
  }
  ;

  /***
   * this method will be called for the user to set the initial P matrix
   */
  virtual void setP(
      Eigen::Matrix<double, EKFState_T::nErrorStatesAtCompileTime,
          EKFState_T::nErrorStatesAtCompileTime>& P) const = 0;

  /***
   * this method will be called for the user to have the possibility to augment the correction vector
   */
  virtual void augmentCorrectionVector(
      Eigen::Matrix<double, EKFState_T::nErrorStatesAtCompileTime, 1>& UNUSEDPARAM(correction)) const {
  }
  ;

  /***
   * this method will be called for the user to check the correction after it has been applied to the state
   * delaystate is the state on which the correction has been applied
   * buffstate is the state before the correction was applied
   */
  virtual void sanityCheckCorrection(
      EKFState_T& UNUSEDPARAM(delaystate),
      const EKFState_T& UNUSEDPARAM(buffstate),
      Eigen::Matrix<double, EKFState_T::nErrorStatesAtCompileTime, 1>& UNUSEDPARAM(correction)) const {
  }
  ;

  /***
   * provide a getter for these parameters, this is implemented for a given middleware or param file parser
   */
  virtual bool getParam_fixed_bias() const = 0;
  virtual double getParam_noise_acc() const = 0;
  virtual double getParam_noise_accbias() const = 0;
  virtual double getParam_noise_gyr() const = 0;
  virtual double getParam_noise_gyrbias() const = 0;
  virtual double getParam_fuzzythres() const = 0;

  /**
   * This functions get called by the core to publish data to external middlewares like ROS
   */
  virtual void publishStateInitial(const shared_ptr<EKFState_T>& state) const = 0;
  virtual void publishStateAfterPropagation(const shared_ptr<EKFState_T>& state) const = 0;
  virtual void publishStateAfterUpdate(const shared_ptr<EKFState_T>& state) const = 0;

};
}
;
// end msf_core

#include <msf_core/implementation/msf_sensormanager.hpp>

#endif /* SENSORMANAGER_H */
