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
#ifndef POSITION_SENSOR_H
#define POSITION_SENSOR_H

#include <msf_core/msf_sensormanagerROS.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <msf_core/gps_conversion.h>
#include <msf_updates/PointWithCovarianceStamped.h>

namespace msf_position_sensor{

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
class PositionSensorHandler : public msf_core::SensorHandler<typename msf_updates::EKFState>
{
private:

  Eigen::Matrix<double, 3, 1> z_p_; ///< position measurement
  double n_zp_; ///< position measurement noise
  double delay_;        ///< delay to be subtracted from the ros-timestamp of the measurement provided by this sensor

  ros::Subscriber subPointStamped_;
  ros::Subscriber subTransformStamped_;
  ros::Subscriber subNavSatFix_;
  msf_core::GPSConversion gpsConversion_;

  bool use_fixed_covariance_; ///< use fixed covariance set by dynamic reconfigure
  bool provides_absolute_measurements_; ///<does this sensor measure relative or absolute values

  void processPositionMeasurement(const msf_updates::PointWithCovarianceStampedConstPtr& msg);
  void measurementCallback(const geometry_msgs::PointStampedConstPtr & msg);
  void measurementCallback(const geometry_msgs::TransformStampedConstPtr & msg);
  void measurementCallback(const sensor_msgs::NavSatFixConstPtr& msg);

public:
  typedef MEASUREMENT_TYPE measurement_t;
  PositionSensorHandler(MANAGER_TYPE& meas, std::string topic_namespace, std::string parameternamespace);
  //used for the init
  Eigen::Matrix<double, 3, 1> getPositionMeasurement(){
    return z_p_;
  }
  //setters for configure values
  void setNoises(double n_zp);
  void setDelay(double delay);
  void adjustGPSZReference(double current_z);

};
}
#include "implementation/position_sensorhandler.hpp"

#endif /* POSITION_SENSOR_H */
