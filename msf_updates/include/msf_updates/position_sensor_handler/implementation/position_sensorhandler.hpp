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
#ifndef POSITION_SENSORHANDLER_HPP_
#define POSITION_SENSORHANDLER_HPP_
#include <msf_core/msf_types.h>
#include <msf_core/eigen_utils.h>
#include <msf_core/gps_conversion.h>

namespace msf_position_sensor {
template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
PositionSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::PositionSensorHandler(
    MANAGER_TYPE& meas, std::string topic_namespace,
    std::string parameternamespace)
    : SensorHandler<msf_updates::EKFState>(meas, topic_namespace,
                                           parameternamespace),
      n_zp_(1e-6),
      delay_(0) {
  ros::NodeHandle pnh("~/position_sensor");
  pnh.param("position_use_fixed_covariance", use_fixed_covariance_, false);
  pnh.param("position_absolute_measurements", provides_absolute_measurements_,
            false);
  pnh.param("enable_mah_outlier_rejection", enable_mah_outlier_rejection_, false);
  pnh.param("mah_threshold", mah_threshold_, msf_core::kDefaultMahThreshold_);

  MSF_INFO_STREAM_COND(use_fixed_covariance_, "Position sensor is using fixed "
                       "covariance");
  MSF_INFO_STREAM_COND(!use_fixed_covariance_, "Position sensor is using "
                       "covariance from sensor");

  MSF_INFO_STREAM_COND(provides_absolute_measurements_, "Position sensor is "
                       "handling measurements as absolute values");
  MSF_INFO_STREAM_COND(!provides_absolute_measurements_, "Position sensor is "
                       "handling measurements as relative values");

  ros::NodeHandle nh("msf_updates");

  subPointStamped_ =
      nh.subscribe<geometry_msgs::PointStamped>
  ("position_input", 20, &PositionSensorHandler::MeasurementCallback, this);
  subTransformStamped_ =
      nh.subscribe<geometry_msgs::TransformStamped>
  ("transform_input", 20, &PositionSensorHandler::MeasurementCallback, this);
  subNavSatFix_ =
      nh.subscribe<sensor_msgs::NavSatFix>
  ("navsatfix_input", 20, &PositionSensorHandler::MeasurementCallback, this);

  z_p_.setZero();

}

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void PositionSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::SetNoises(
    double n_zp) {
  n_zp_ = n_zp;
}

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void PositionSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::SetDelay(
    double delay) {
  delay_ = delay;
}

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void PositionSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::AdjustGPSZReference(
    double current_z) {
  gpsConversion_.AdjustReference(z_p_(2) - current_z);
}

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void PositionSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::ProcessPositionMeasurement(
    const sensor_fusion_comm::PointWithCovarianceStampedConstPtr& msg) {
  received_first_measurement_ = true;

  // Get the fixed states.
  int fixedstates = 0;
  static_assert(msf_updates::EKFState::nStateVarsAtCompileTime < 32, "Your state "
      "has more than 32 variables. The code needs to be changed here to have a "
      "larger variable to mark the fixed_states");
  // Do not exceed the 32 bits of int.

  if (!use_fixed_covariance_ && msg->covariance[0] == 0)  // Take covariance from sensor.
      {
    MSF_WARN_STREAM_THROTTLE(
        2, "Provided message type without covariance but set "
        "fixed_covariance=false at the same time. Discarding message.");
    return;
  }

  // Get all the fixed states and set flag bits.
  MANAGER_TYPE* mngr = dynamic_cast<MANAGER_TYPE*>(&manager_);

  if (mngr) {
    if (mngr->Getcfg().position_fixed_p_ip) {
      fixedstates |= 1 << msf_updates::EKFState::StateDefinition_T::p_ip;
    }
  }

  shared_ptr<MEASUREMENT_TYPE> meas(new MEASUREMENT_TYPE(
      n_zp_, use_fixed_covariance_, provides_absolute_measurements_,
      this->sensorID, fixedstates, enable_mah_outlier_rejection_,
      mah_threshold_));

  meas->MakeFromSensorReading(msg, msg->header.stamp.toSec() - delay_);

  z_p_ = meas->z_p_;  // Store this for the init procedure.

  this->manager_.msf_core_->AddMeasurement(meas);
}

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void PositionSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::MeasurementCallback(
    const geometry_msgs::PointStampedConstPtr & msg) {
  this->SequenceWatchDog(msg->header.seq, subPointStamped_.getTopic());

  MSF_INFO_STREAM_ONCE(
      "*** position sensor got first measurement from topic "
          << this->topic_namespace_ << "/" << subPointStamped_.getTopic()
          << " ***");

  sensor_fusion_comm::PointWithCovarianceStampedPtr pointwCov(
      new sensor_fusion_comm::PointWithCovarianceStamped);
  pointwCov->header = msg->header;
  pointwCov->point = msg->point;

  ProcessPositionMeasurement(pointwCov);
}

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void PositionSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::MeasurementCallback(
    const geometry_msgs::TransformStampedConstPtr & msg) {
  this->SequenceWatchDog(msg->header.seq, subTransformStamped_.getTopic());

  MSF_INFO_STREAM_ONCE(
      "*** position sensor got first measurement from topic "
          << this->topic_namespace_ << "/" << subTransformStamped_.getTopic()
          << " ***");

  if (msg->header.seq % 5 != 0) {  //slow down vicon
    MSF_WARN_STREAM_ONCE("Measurement throttling is on, dropping every but the "
                         "5th message");
    return;
  }

  sensor_fusion_comm::PointWithCovarianceStampedPtr pointwCov(
      new sensor_fusion_comm::PointWithCovarianceStamped);
  pointwCov->header = msg->header;

  // Fixed covariance will be set in measurement class -> MakeFromSensorReadingImpl.
  pointwCov->point.x = msg->transform.translation.x;
  pointwCov->point.y = msg->transform.translation.y;
  pointwCov->point.z = msg->transform.translation.z;

  ProcessPositionMeasurement(pointwCov);
}

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void PositionSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::MeasurementCallback(
    const sensor_msgs::NavSatFixConstPtr& msg) {
  this->SequenceWatchDog(msg->header.seq, subNavSatFix_.getTopic());

  MSF_INFO_STREAM_ONCE(
      "*** position sensor got first measurement from topic "
          << this->topic_namespace_ << "/" << subNavSatFix_.getTopic()
          << " ***");

  // Fixed covariance will be set in measurement class -> MakeFromSensorReadingImpl.
  static bool referenceinit = false;  //TODO (slynen): Dynreconf reset reference.
  if (!referenceinit) {
    gpsConversion_.InitReference(msg->latitude, msg->longitude, msg->altitude);
    MSF_WARN_STREAM(
        "Initialized GPS reference of topic: " << this->topic_namespace_ << "/"
            << subNavSatFix_.getTopic() << " to lat/lon/alt: [" << msg->latitude
            << ", " << msg->longitude << ", " << msg->altitude << "]");
    referenceinit = true;
  }

  msf_core::Vector3 ecef = gpsConversion_.WGS84ToECEF(msg->latitude,
                                                      msg->longitude,
                                                      msg->altitude);
  msf_core::Vector3 enu = gpsConversion_.ECEFToENU(ecef);

  sensor_fusion_comm::PointWithCovarianceStampedPtr pointwCov(
      new sensor_fusion_comm::PointWithCovarianceStamped);
  pointwCov->header = msg->header;

  // Store the ENU data in the position fields.
  pointwCov->point.x = enu[0];
  pointwCov->point.y = enu[1];
  pointwCov->point.z = enu[2];

  // Get the covariance TODO (slynen): handle the cases differently!
  if (msg->position_covariance_type
      == sensor_msgs::NavSatFix::COVARIANCE_TYPE_KNOWN) {
    pointwCov->covariance = msg->position_covariance;
  } else if (msg->position_covariance_type
      == sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN) {
    pointwCov->covariance = msg->position_covariance;
  } else if (msg->position_covariance_type
      == sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED) {  // From DOP.
    pointwCov->covariance = msg->position_covariance;
  }

  ProcessPositionMeasurement(pointwCov);
}
}  // namespace msf_position_sensor
#endif  // POSITION_SENSORHANDLER_HPP_
