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

#ifndef POSE_SENSOR_H
#define POSE_SENSOR_H

#include <msf_core/msf_sensormanagerROS.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <msf_updates/PoseDistorter.h>

namespace msf_pose_sensor{

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
class PoseSensorHandler : public msf_core::SensorHandler<typename msf_updates::EKFState>
{
private:

  Eigen::Quaternion<double> z_q_; ///< attitude measurement camera seen from world
  Eigen::Matrix<double, 3, 1> z_p_; ///< position measurement camera seen from world
  double n_zp_, n_zq_; ///< position and attitude measurement noise
  double delay_;        ///< delay to be subtracted from the ros-timestamp of the measurement provided by this sensor

  ros::Subscriber subPoseWithCovarianceStamped_;
  ros::Subscriber subTransformStamped_;
  ros::Subscriber subPoseStamped_;

  bool measurement_world_sensor_; ///< defines if the pose of the sensor is measured in world coordinates (true, default) or vice versa (false, e.g. PTAM)
  bool use_fixed_covariance_; ///< use fixed covariance set by dynamic reconfigure
  bool provides_absolute_measurements_; ///<does this sensor measure relative or absolute values

  msf_updates::PoseDistorter::Ptr distorter_;

  void ProcessPoseMeasurement(const geometry_msgs::PoseWithCovarianceStampedConstPtr & msg);
  void measurementCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr & msg);
  void measurementCallback(const geometry_msgs::PoseStampedConstPtr & msg);
  void measurementCallback(const geometry_msgs::TransformStampedConstPtr & msg);

public:
  typedef MEASUREMENT_TYPE measurement_t;
  PoseSensorHandler(MANAGER_TYPE& meas, std::string topic_namespace, std::string parameternamespace, bool distortmeas);
  //used for the init
  Eigen::Matrix<double, 3, 1> getPositionMeasurement(){
    return z_p_;
  }
  Eigen::Quaterniond getAttitudeMeasurement(){
    return z_q_;
  }
  //setters for configure values
  void setNoises(double n_zp, double n_zq);
  void setDelay(double delay);

};
}
#include "implementation/pose_sensorhandler.hpp"

#endif /* POSE_SENSOR_H */
