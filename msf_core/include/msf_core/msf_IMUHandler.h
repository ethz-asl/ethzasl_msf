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
#ifndef MSF_IMU_HANDLER_H_
#define MSF_IMU_HANDLER_H_

#include <msf_core/msf_sensorhandler.h>

namespace msf_core{

template<typename EKFState_T>
class IMUHandler:public SensorHandler<EKFState_T>{
 protected:
  shared_ptr<MSF_Core<EKFState_T> > core_;
public:
  IMUHandler(MSF_SensorManager<EKFState_T>& mng,
             const std::string& topic_namespace, const std::string& parameternamespace):
               SensorHandler<EKFState_T>(mng, topic_namespace, parameternamespace){
    core_ = mng.msf_core_;
  }
  virtual ~IMUHandler(){};
  virtual bool initialize() = 0;
  void process_imu(const msf_core::Vector3& linear_acceleration,
      const msf_core::Vector3& angular_velocity, const double& msg_stamp,
      size_t msg_seq){
    core_->process_imu(linear_acceleration, angular_velocity, msg_stamp, msg_seq);
  }
  void process_state(const msf_core::Vector3& linear_acceleration,
      const msf_core::Vector3& angular_velocity, const msf_core::Vector3& p,
      const msf_core::Vector3& v, const msf_core::Quaternion& q, bool is_already_propagated, const double& msg_stamp,
      size_t msg_seq){
    core_->process_extstate(linear_acceleration, angular_velocity, p, v, q, is_already_propagated, msg_stamp, msg_seq);
  }
};
}

#endif /* MSF_IMU_HANDLER_H_ */
