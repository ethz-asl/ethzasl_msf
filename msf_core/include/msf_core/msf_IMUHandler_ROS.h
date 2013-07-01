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
#ifndef MSF_IMUHANDLER_ROS_H_
#define MSF_IMUHANDLER_ROS_H_

#include <msf_core/msf_IMUHandler.h>

namespace msf_core{

template<typename EKFState_T>
class IMUHandler_ROS:public IMUHandler<EKFState_T>{
  ros::Subscriber subState_; ///< subscriber to external state propagation
  ros::Subscriber subImu_; ///< subscriber to IMU readings
  ros::Subscriber subImuCustom_; ///< subscriber to IMU readings for asctec custom

 public:
  IMUHandler_ROS(MSF_SensorManager<EKFState_T>& mng,
                 const std::string& topic_namespace, const std::string& parameternamespace):
                   IMUHandler<EKFState_T>(mng, topic_namespace, parameternamespace){

    ros::NodeHandle nh(topic_namespace);

    subImu_ = nh.subscribe("imu_state_input", 100, &IMUHandler_ROS::imuCallback, this);
    subImuCustom_ = nh.subscribe("imu_state_input_asctec", 10,
                                 &IMUHandler_ROS::imuCallback_asctec, this);
    subState_ = nh.subscribe("hl_state_input", 10, &IMUHandler_ROS::stateCallback, this);
  }

  virtual ~IMUHandler_ROS(){};

  void stateCallback(const sensor_fusion_comm::ExtEkfConstPtr & msg) {

    static_cast<MSF_SensorManagerROS<EKFState_T>& >(this->manager_).setHLStateBuffer(*msg);

    //get the imu values
    msf_core::Vector3 linacc;
    linacc << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;

    msf_core::Vector3 angvel;
    angvel << msg->angular_velocity.x, msg->angular_velocity.y, msg
        ->angular_velocity.z;

    int32_t flag = msg->flag;
    //make sure we tell the HL to ignore if data playback is on
    if (this->manager_.data_playback())
      flag = sensor_fusion_comm::ExtEkf::ignore_state;

    bool isnumeric = true;
    if (flag == sensor_fusion_comm::ExtEkf::current_state){
      isnumeric = checkForNumeric(
          Eigen::Map<const Eigen::Matrix<float, 10, 1> >(msg->state.data()),
          "before prediction p,v,q");
    }

    //get the propagated states
    msf_core::Vector3 p, v;
    msf_core::Quaternion q;

    p = Eigen::Matrix<double, 3, 1>(msg->state[0], msg->state[1], msg->state[2]);
    v = Eigen::Matrix<double, 3, 1>(msg->state[3], msg->state[4], msg->state[5]);
    q = Eigen::Quaternion<double>(msg->state[6], msg->state[7], msg->state[8], msg->state[9]);
    q.normalize();

    bool is_already_propagated = false;
    if(flag == sensor_fusion_comm::ExtEkf::current_state && isnumeric){
      is_already_propagated = true;
    }

    this->process_state(linacc, angvel, p, v, q, is_already_propagated, msg->header.stamp.toSec(), msg->header.seq);
  }

  void imuCallback_asctec(
      const asctec_hl_comm::mav_imuConstPtr & msg) {

    msf_core::Vector3 linacc;
    linacc << msg->acceleration.x, msg->acceleration.y, msg->acceleration.z;

    msf_core::Vector3 angvel;
    angvel << msg->angular_velocity.x, msg->angular_velocity.y, msg
        ->angular_velocity.z;

    this->process_imu(linacc, angvel, msg->header.stamp.toSec(), msg->header.seq);

  }

  void imuCallback(const sensor_msgs::ImuConstPtr & msg) {
    static int lastseq = -1;
    if ((int) msg->header.seq != lastseq + 1 && lastseq != -1) {
      MSF_WARN_STREAM(
          "msf_core: imu message drop curr seq:" << msg->header.seq << " expected: " << lastseq + 1);
    }
    lastseq = msg->header.seq;

    msf_core::Vector3 linacc;
    linacc << msg->linear_acceleration.x, msg->linear_acceleration.y, msg
        ->linear_acceleration.z;

    msf_core::Vector3 angvel;
    angvel << msg->angular_velocity.x, msg->angular_velocity.y, msg
        ->angular_velocity.z;

    this->process_imu(linacc, angvel, msg->header.stamp.toSec(), msg->header.seq);
  }

  virtual bool initialize(){
    return true;
  }
};
}


#endif /* MSF_IMUHANDLER_ROS_H_ */
