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
#ifndef POSE_SENSOR_H_
#define POSE_SENSOR_H_

#include <msf_core/msf_sensormanagerROS.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <msf_updates/PoseDistorter.h>

namespace msf_pose_sensor {

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
class PoseSensorHandler : public msf_core::SensorHandler<
    typename msf_updates::EKFState> {
 private:

  Eigen::Quaternion<double> z_q_;  ///< Attitude measurement camera seen from world.
  Eigen::Matrix<double, 3, 1> z_p_;  ///< Position measurement camera seen from world.
  double n_zp_, n_zq_;  ///< Position and attitude measurement noise.
  double delay_;        ///< Delay to be subtracted from the ros-timestamp of
  // the measurement provided by this sensor.

  ros::Subscriber subPoseWithCovarianceStamped_;
  ros::Subscriber subTransformStamped_;
  ros::Subscriber subPoseStamped_;

  bool measurement_world_sensor_;  ///< Defines if the pose of the sensor is
                                   // measured in world coordinates (true, default)
                                   // or vice versa (false, e.g. PTAM)
  bool use_fixed_covariance_;  ///< Use fixed covariance set by dynamic reconfigure
  bool provides_absolute_measurements_;  ///<Does this sensor measure relative or
                                         // absolute values

  double pose_measurement_minimum_dt_; ///< Minimum time between two pose measurements in seconds.
                                       // If more pose measurements are received they are dropped.

  double timestamp_previous_pose_;  ///< Timestamp of previous pose message to subsample messages.

  msf_updates::PoseDistorter::Ptr distorter_;

  void ProcessPoseMeasurement(
      const geometry_msgs::PoseWithCovarianceStampedConstPtr & msg);
  void MeasurementCallback(
      const geometry_msgs::PoseWithCovarianceStampedConstPtr & msg);
  void MeasurementCallback(const geometry_msgs::PoseStampedConstPtr & msg);
  void MeasurementCallback(const geometry_msgs::TransformStampedConstPtr & msg);

 public:
  typedef MEASUREMENT_TYPE measurement_t;
  PoseSensorHandler(MANAGER_TYPE& meas, std::string topic_namespace,
                    std::string parameternamespace, bool distortmeas);
  // Used for the init.
  Eigen::Matrix<double, 3, 1> GetPositionMeasurement() {
    return z_p_;
  }
  Eigen::Quaterniond GetAttitudeMeasurement() {
    return z_q_;
  }
  //setters for configure values
  void SetNoises(double n_zp, double n_zq);
  void SetDelay(double delay);
};
}  // namespace msf_pose_sensor
#include "implementation/pose_sensorhandler.hpp"

#endif  // POSE_SENSOR_H_
