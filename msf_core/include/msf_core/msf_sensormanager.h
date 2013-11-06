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
#ifndef SENSORMANAGER_H
#define SENSORMANAGER_H

#include <Eigen/Dense>
#include <string.h>
#include <msf_core/msf_types.h>
#include <msf_core/msf_statevisitor.h>
#include <msf_core/msf_macros.h>

namespace msf_core {

template<typename EKFState_T>
class SensorHandler;

template<typename EKFState_T>
class MSF_Core;

/** \class MSF_SensorManager
 * \brief A manager for a given sensor set. Handlers for individual sensors
 * (camera/vicon etc.) are registered with this class as handlers of particular
 * sensors. This class also owns the EKF core instance and handles the
 * initialization of the filter.
 */
template<typename EKFState_T>
class MSF_SensorManager : public StateVisitor<EKFState_T> {
 private:
  int sensorID_;
 protected:
  typedef std::vector<shared_ptr<SensorHandler<EKFState_T> > > Handlers;
  Handlers handlers;  ///< A list of sensor handlers which provide measurements.

  /**
   * Used to determine if internal states get overwritten by the external
   * state prediction (online) or internal state prediction is performed
   * for log replay, when the external prediction is not available or should be
   * done on the host.
   */
  bool data_playback_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  shared_ptr<MSF_Core<EKFState_T> > msf_core_;  ///< The ekf core instance.

  MSF_SensorManager();

  bool GetDataPlaybackStatus() {
    return data_playback_;
  }

  virtual ~MSF_SensorManager() {

  }
  ;
  /***
   * Add a new sensor handler to the list of handlers owned by this manager
   * a sensor handler is in turn owning the sensor (camera/vicon etc.).
   */
  void AddHandler(shared_ptr<SensorHandler<EKFState_T> > handler) {
    handler->SetSensorID(sensorID_++);
    handlers.push_back(handler);
  }

  /***
   * Init function for the EKF.
   */
  virtual void Init(double scale) const = 0;

  /***
   * This method will be called for the user to set the initial state.
   */
  virtual void InitState(EKFState_T& state) const = 0;

  /***
   * This method will be called for the user to set the Q block entries for
   * Auxiliary states only changes to blocks in Q belonging to the auxiliary
   * states are allowed / evaluated.
   */
  virtual void CalculateQAuxiliaryStates(EKFState_T& UNUSEDPARAM(state),
                                         double UNUSEDPARAM(dt)) const = 0;

  /***
   * This method will be called for the user to set the initial P matrix.
   */
  virtual void SetStateCovariance(
      Eigen::Matrix<double, EKFState_T::nErrorStatesAtCompileTime,
          EKFState_T::nErrorStatesAtCompileTime>& P) const = 0;

  /***
   * This method will be called for the user to have the possibility to augment
   * the correction vector.
   */
  virtual void AugmentCorrectionVector(
      Eigen::Matrix<double, EKFState_T::nErrorStatesAtCompileTime, 1>&
      UNUSEDPARAM(correction)) const = 0;

  /***
   * This method will be called for the user to check the correction after it
   * has been applied to the state delaystate is the state on which the correction
   * has been applied buffstate is the state before the correction was applied.
   */
  virtual void SanityCheckCorrection(
      EKFState_T& UNUSEDPARAM(delaystate),
      const EKFState_T& UNUSEDPARAM(buffstate),
      Eigen::Matrix<double, EKFState_T::nErrorStatesAtCompileTime, 1>&
      UNUSEDPARAM(correction)) const  = 0;

  /***
   * Provide a getter for these parameters, this is implemented for a given
   * middleware or param file parser.
   */
  virtual bool GetParamFixedBias() const = 0;
  virtual double GetParamNoiseAcc() const = 0;
  virtual double GetParamNoiseAccbias() const = 0;
  virtual double GetParamNoiseGyr() const = 0;
  virtual double GetParamNoiseGyrbias() const = 0;
  virtual double GetParamFuzzyTrackingThreshold() const = 0;

  /**
   * This functions get called by the core to publish data to external
   * middlewares like ROS.
   */
  virtual void PublishStateInitial(
      const shared_ptr<EKFState_T>& state) const = 0;
  virtual void PublishStateAfterPropagation(
      const shared_ptr<EKFState_T>& state) const = 0;
  virtual void PublishStateAfterUpdate(
      const shared_ptr<EKFState_T>& state) const = 0;

};
}  // namespace msf_core

#include <msf_core/implementation/msf_sensormanager_inl.h>

#endif  // SENSORMANAGER_H
