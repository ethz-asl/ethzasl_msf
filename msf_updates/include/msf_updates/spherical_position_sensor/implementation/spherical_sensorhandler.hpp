/*
 * Copyright (C) 2012-2013 Simon Lynen, ASL, ETH Zurich, Switzerland
 * You can contact the author at <slynen at ethz dot ch>
 * Copyright (C) 2013 Georg Wiedebach, ASL, ETH Zurich, Switzerland
 * You can contact the author at <georgwi at ethz dot ch>
 * Copyright (c) 2012, Markus Achtelik, ASL, ETH Zurich, Switzerland
 * You can contact the author at <acmarkus at ethz dot ch>
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
#include <msf_core/eigen_utils.h>
#include <msf_core/gps_conversion.h>
#ifndef SPHERICAL_SENSORHANDLER_HPP_
#define SPHERICAL_SENSORHANDLER_HPP_

namespace msf_spherical_position {
template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
AngleSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::AngleSensorHandler(
    MANAGER_TYPE& meas, std::string topic_namespace,
    std::string parameternamespace)
    : SensorHandler<msf_updates::EKFState>(meas, topic_namespace,
                                           parameternamespace),
      n_za_(1e-6),
      delay_(0) {
  ros::NodeHandle pnh("~/spherical_position_sensor");
  pnh.param("use_fixed_covariance", use_fixed_covariance_, true);
  pnh.param("absolute_measurements", provides_absolute_measurements_, false);
  pnh.param("enable_mah_outlier_rejection", enable_mah_outlier_rejection_, false);
  pnh.param("mah_threshold", mah_threshold_, msf_core::kDefaultMahThreshold_);

  ROS_INFO_COND(use_fixed_covariance_,
                "Angle sensor is using fixed covariance");
  ROS_INFO_COND(!use_fixed_covariance_,
                "Angle sensor is using covariance from sensor");

  ROS_INFO_COND(provides_absolute_measurements_,
                "Angle sensor is handling measurements as absolute values");
  ROS_INFO_COND(!provides_absolute_measurements_,
                "Angle sensor is handling measurements as relative values");

  ros::NodeHandle nh("msf_updates");

  subPointStamped_ = nh.subscribe<geometry_msgs::PointStamped>
      ("angle_input", 20, &AngleSensorHandler::MeasurementCallback, this);

  z_a_.setZero();

}

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void AngleSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::SetNoises(
    double n_za) {
  n_za_ = n_za;
}

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void AngleSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::SetDelay(
    double delay) {
  delay_ = delay;
}

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void AngleSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::MeasurementCallback(
    const geometry_msgs::PointStampedConstPtr & msg) {
  received_first_measurement_ = true;

  this->SequenceWatchDog(msg->header.seq, subPointStamped_.getTopic());

  ROS_INFO_STREAM_ONCE(
      "*** Angle sensor got first measurement from topic "
          << this->topic_namespace_ << "/" << subPointStamped_.getTopic()
          << " ***");

  //get the fixed states
  int fixedstates = 0;
  BOOST_STATIC_ASSERT_MSG(
      msf_updates::EKFState::nStateVarsAtCompileTime < 32,
      "Your state has more than 32 variables. "
      "The code needs to be changed here to have a larger variable to mark the "
      "fixed_states");  //do not exceed the 32 bits of int

  if (!use_fixed_covariance_ /*&& msg->covariance[0] == 0*/)  // take covariance from sensor
  {
    ROS_WARN_STREAM_THROTTLE(
        2,
        "Provided message type without covariance but set fixed_covariance=false "
        "at the same time. Discarding message.");
    return;
  }

  //get all the fixed states and set flag bits
  MANAGER_TYPE* mngr = dynamic_cast<MANAGER_TYPE*>(&manager_);

  if (mngr) {
    if (mngr->Getcfg().fixed_p_ip) {
      fixedstates |= 1 << msf_updates::EKFState::StateDefinition_T::p_ip;
    }
  }

  typename boost::shared_ptr<MEASUREMENT_TYPE> meas(new MEASUREMENT_TYPE(
      n_za_, use_fixed_covariance_, provides_absolute_measurements_,
      this->sensorID, fixedstates, enable_mah_outlier_rejection_,
      mah_threshold_));

  meas->MakeFromSensorReading(msg, msg->header.stamp.toSec() - delay_);

  z_a_ = meas->z_a_;  //store this for the init procedure

  this->manager_.msf_core_->AddMeasurement(meas);
}

// Distance sensor implementation:
template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
DistanceSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::DistanceSensorHandler(
    MANAGER_TYPE& meas, std::string topic_namespace,
    std::string parameternamespace)
    : SensorHandler<msf_updates::EKFState>(meas, topic_namespace,
                                           parameternamespace),
      n_zd_(1e-6),
      delay_(0) {
  ros::NodeHandle pnh("~/spherical_position_sensor");
  pnh.param("use_fixed_covariance", use_fixed_covariance_, true);
  pnh.param("absolute_measurements", provides_absolute_measurements_, false);
  pnh.param("enable_mah_outlier_rejection", enable_mah_outlier_rejection_, false);
  pnh.param("mah_threshold", mah_threshold_, msf_core::kDefaultMahThreshold_);

  ROS_INFO_COND(use_fixed_covariance_,
                "Distance sensor is using fixed covariance");
  ROS_INFO_COND(!use_fixed_covariance_,
                "Distance sensor is using covariance from sensor");

  ROS_INFO_COND(provides_absolute_measurements_,
                "Distance sensor is handling measurements as absolute values");
  ROS_INFO_COND(!provides_absolute_measurements_,
                "Distance sensor is handling measurements as relative values");

  ros::NodeHandle nh("msf_updates");

  subPointStamped_ =
      nh.subscribe<geometry_msgs::PointStamped>
      ("distance_input", 20, &DistanceSensorHandler::MeasurementCallback, this);

  z_d_.setZero();

}

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void DistanceSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::SetNoises(
    double n_za) {
  n_zd_ = n_za;
}

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void DistanceSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::SetDelay(
    double delay) {
  delay_ = delay;
}

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void DistanceSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::MeasurementCallback(
    const geometry_msgs::PointStampedConstPtr & msg) {
  received_first_measurement_ = true;

  this->SequenceWatchDog(msg->header.seq, subPointStamped_.getTopic());

  ROS_INFO_STREAM_ONCE(
      "*** Distance sensor got first measurement from topic "
          << this->topic_namespace_ << "/" << subPointStamped_.getTopic()
          << " ***");

  //get the fixed states
  int fixedstates = 0;
  BOOST_STATIC_ASSERT_MSG(
      msf_updates::EKFState::nStateVarsAtCompileTime < 32,
      "Your state has more than 32 variables. "
      "The code needs to be changed here to have a larger variable to mark the fixed_states");  //do not exceed the 32 bits of int

  if (!use_fixed_covariance_) {
    ROS_WARN_STREAM_THROTTLE(
        2,
        "Provided message type without covariance but set fixed_covariance=false at the same time. Discarding message.");
    return;
  }

  //get all the fixed states and set flag bits
  MANAGER_TYPE* mngr = dynamic_cast<MANAGER_TYPE*>(&manager_);

  if (mngr) {
    if (mngr->Getcfg().fixed_p_ip) {
      fixedstates |= 1 << msf_updates::EKFState::StateDefinition_T::p_ip;
    }
  }

  typename boost::shared_ptr<MEASUREMENT_TYPE> meas(new MEASUREMENT_TYPE(
      n_zd_, use_fixed_covariance_, provides_absolute_measurements_,
      this->sensorID, fixedstates, enable_mah_outlier_rejection_,
      mah_threshold_));

  meas->MakeFromSensorReading(msg, msg->header.stamp.toSec() - delay_);

  z_d_ = meas->z_d_;  //store this for the init procedure

  this->manager_.msf_core_->AddMeasurement(meas);
}
}  // namespace msf_spherical_position
#endif  // SPHERICAL_SENSORHANDLER_HPP_
