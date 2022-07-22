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

#ifndef VELOCITY_MEASUREMENTMANAGER_H_
#define VELOCITY_MEASUREMENTMANAGER_H_

#include <ros/ros.h>

#include <msf_core/msf_core.h>
#include <msf_core/msf_sensormanagerROS.h>
#include <msf_core/msf_IMUHandler_ROS.h>
#include "msf_statedef.hpp"
#include <msf_updates/velocity_sensor_handler/velocity_sensorhandler.h>
#include <msf_updates/velocity_sensor_handler/velocity_xy_measurement.h>


namespace msf_velocity_sensor {
// TODO: Change to velocity
// typedef msf_updates::SinglePositionSensorConfig Config_T;

class VelocitySensorManager
    : public msf_core::MSF_SensorManagerROS<msf_updates::EKFState> {
 public:
  typedef msf_updates::EKFState EKFState_T;
  typedef EKFState_T::StateSequence_T StateSequence_T;
  typedef EKFState_T::StateDefinition_T StateDefinition_T;

  VelocitySensorManager(
      ros::NodeHandle pnh = ros::NodeHandle("~/velocity_sensor")) {}

  virtual ~VelocitySensorManager() {}

  // virtual const Config_T& Getcfg() { return config_; }

 private:
  // Config_T config_;

  // TODO:
  // virtual void Config(Config_T& config, uint32_t level) {}
  void Init(double scale) const {}
  virtual void ResetState(EKFState_T& state) const {}
  virtual void InitState(EKFState_T& state) const {}
  virtual void CalculateQAuxiliaryStates(EKFState_T& state, double dt) const {}

  virtual void SetStateCovariance(
      Eigen::Matrix<double, EKFState_T::nErrorStatesAtCompileTime,
                    EKFState_T::nErrorStatesAtCompileTime>& P) const {}

  virtual void AugmentCorrectionVector(
      Eigen::Matrix<double, EKFState_T::nErrorStatesAtCompileTime, 1>&
          correction) const {}

  virtual void SanityCheckCorrection(
      EKFState_T& delayedstate, const EKFState_T& buffstate,
      Eigen::Matrix<double, EKFState_T::nErrorStatesAtCompileTime, 1>&
          correction) const {}
};
}  // namespace msf_velocity_sensor

#endif  // VELOCITY_MEASUREMENTMANAGER_H_
