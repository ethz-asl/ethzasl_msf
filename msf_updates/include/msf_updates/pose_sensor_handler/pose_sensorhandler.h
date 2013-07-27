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
