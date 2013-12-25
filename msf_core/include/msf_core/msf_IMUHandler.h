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
#ifndef MSF_IMU_HANDLER_H_
#define MSF_IMU_HANDLER_H_

#include <msf_core/msf_sensorhandler.h>

namespace msf_core {

template<typename EKFState_T>
class IMUHandler : public SensorHandler<EKFState_T> {
 protected:
  shared_ptr<MSF_Core<EKFState_T> > core_;
 public:
  IMUHandler(MSF_SensorManager<EKFState_T>& mng,
             const std::string& topic_namespace,
             const std::string& parameternamespace)
      : SensorHandler<EKFState_T>(mng, topic_namespace, parameternamespace) {
    core_ = mng.msf_core_;
  }
  virtual ~IMUHandler() {
  }
  ;
  virtual bool Initialize() = 0;
  void ProcessIMU(const msf_core::Vector3& linear_acceleration,
                   const msf_core::Vector3& angular_velocity,
                   const double& msg_stamp, size_t msg_seq) {
    core_->ProcessIMU(linear_acceleration, angular_velocity, msg_stamp,
                       msg_seq);
  }
  void ProcessState(const msf_core::Vector3& linear_acceleration,
                     const msf_core::Vector3& angular_velocity,
                     const msf_core::Vector3& p, const msf_core::Vector3& v,
                     const msf_core::Quaternion& q, bool is_already_propagated,
                     const double& msg_stamp, size_t msg_seq) {
    core_->ProcessExternallyPropagatedState(linear_acceleration,
                                            angular_velocity, p, v, q,
                                            is_already_propagated,
                                            msg_stamp, msg_seq);
  }
};
}

#endif  // MSF_IMU_HANDLER_H_
