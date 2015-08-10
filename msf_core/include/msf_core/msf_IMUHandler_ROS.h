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
#ifndef MSF_IMUHANDLER_ROS_H_
#define MSF_IMUHANDLER_ROS_H_

#include <msf_core/msf_IMUHandler.h>

namespace msf_core {

template<typename EKFState_T>
class IMUHandler_ROS : public IMUHandler<EKFState_T> {
  ros::Subscriber subState_;  ///< subscriber to external state propagation
  ros::Subscriber subImu_;  ///< subscriber to IMU readings
  ros::Subscriber subImuCustom_;  ///< subscriber to IMU readings for asctec custom

 public:
  IMUHandler_ROS(MSF_SensorManager<EKFState_T>& mng,
                 const std::string& topic_namespace,
                 const std::string& parameternamespace)
      : IMUHandler<EKFState_T>(mng, topic_namespace, parameternamespace) {

    ros::NodeHandle nh(topic_namespace);

    subImu_ = nh.subscribe("imu_state_input", 100, &IMUHandler_ROS::IMUCallback,
                           this);
    subState_ = nh.subscribe("hl_state_input", 10,
                             &IMUHandler_ROS::StateCallback, this);
  }

  virtual ~IMUHandler_ROS() { }

  void StateCallback(const sensor_fusion_comm::ExtEkfConstPtr & msg) {
    static_cast<MSF_SensorManagerROS<EKFState_T>&>(this->manager_)
        .SetHLControllerStateBuffer(*msg);

    // Get the imu values.
    msf_core::Vector3 linacc;
    linacc << msg->linear_acceleration.x, msg->linear_acceleration.y, msg
        ->linear_acceleration.z;

    msf_core::Vector3 angvel;
    angvel << msg->angular_velocity.x, msg->angular_velocity.y, msg
        ->angular_velocity.z;

    int32_t flag = msg->flag;
    // Make sure we tell the HL to ignore if data playback is on.
    if (this->manager_.GetDataPlaybackStatus())
      flag = sensor_fusion_comm::ExtEkf::ignore_state;

    bool isnumeric = true;
    if (flag == sensor_fusion_comm::ExtEkf::current_state) {
      isnumeric = CheckForNumeric(
          Eigen::Map<const Eigen::Matrix<float, 10, 1> >(msg->state.data()),
          "before prediction p,v,q");
    }

    // Get the propagated states.
    msf_core::Vector3 p, v;
    msf_core::Quaternion q;

    p = Eigen::Matrix<double, 3, 1>(msg->state[0], msg->state[1],
                                    msg->state[2]);
    v = Eigen::Matrix<double, 3, 1>(msg->state[3], msg->state[4],
                                    msg->state[5]);
    q = Eigen::Quaternion<double>(msg->state[6], msg->state[7], msg->state[8],
                                  msg->state[9]);
    q.normalize();

    bool is_already_propagated = false;
    if (flag == sensor_fusion_comm::ExtEkf::current_state && isnumeric) {
      is_already_propagated = true;
    }

    this->ProcessState(linacc, angvel, p, v, q, is_already_propagated,
                        msg->header.stamp.toSec(), msg->header.seq);
  }

  void IMUCallback(const sensor_msgs::ImuConstPtr & msg) {
    static int lastseq = constants::INVALID_SEQUENCE;
    if (static_cast<int>(msg->header.seq) != lastseq + 1
        && lastseq != constants::INVALID_SEQUENCE) {
      MSF_WARN_STREAM(
          "msf_core: imu message drop curr seq:" << msg->header.seq
              << " expected: " << lastseq + 1);
    }
    lastseq = msg->header.seq;

    msf_core::Vector3 linacc;
    linacc << msg->linear_acceleration.x, msg->linear_acceleration.y, msg
        ->linear_acceleration.z;

    msf_core::Vector3 angvel;
    angvel << msg->angular_velocity.x, msg->angular_velocity.y, msg
        ->angular_velocity.z;

    this->ProcessIMU(linacc, angvel, msg->header.stamp.toSec(),
                      msg->header.seq);
  }

  virtual bool Initialize() {
    return true;
  }
};
}

#endif  // MSF_IMUHANDLER_ROS_H_
