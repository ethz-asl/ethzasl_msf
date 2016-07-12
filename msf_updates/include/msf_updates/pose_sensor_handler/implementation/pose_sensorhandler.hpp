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
#include <msf_core/eigen_utils.h>
#include <msf_core/msf_types.h>

#ifndef POSE_SENSORHANDLER_HPP_
#define POSE_SENSORHANDLER_HPP_

namespace msf_pose_sensor {
template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
PoseSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::PoseSensorHandler(
    MANAGER_TYPE& meas, std::string topic_namespace,
    std::string parameternamespace, bool distortmeas)
    : SensorHandler<msf_updates::EKFState>(meas, topic_namespace,
                                           parameternamespace),
      n_zp_(1e-6),
      n_zq_(1e-6),
      delay_(0),
      timestamp_previous_pose_(0) {
  ros::NodeHandle pnh("~/" + parameternamespace);

  MSF_INFO_STREAM(
      "Loading parameters for pose sensor from namespace: "
          << pnh.getNamespace());

  pnh.param("pose_absolute_measurements", provides_absolute_measurements_,
            true);
  pnh.param("pose_measurement_world_sensor", measurement_world_sensor_, true);
  pnh.param("pose_use_fixed_covariance", use_fixed_covariance_, false);
  pnh.param("pose_measurement_minimum_dt", pose_measurement_minimum_dt_, 0.05);
  pnh.param("enable_mah_outlier_rejection", enable_mah_outlier_rejection_, false);
  pnh.param("mah_threshold", mah_threshold_, msf_core::kDefaultMahThreshold_);

  MSF_INFO_STREAM_COND(measurement_world_sensor_, "Pose sensor is interpreting "
                       "measurement as sensor w.r.t. world");
  MSF_INFO_STREAM_COND(
      !measurement_world_sensor_,
      "Pose sensor is interpreting measurement as world w.r.t. "
      "sensor (e.g. ethzasl_ptam)");

  MSF_INFO_STREAM_COND(use_fixed_covariance_, "Pose sensor is using fixed "
                       "covariance");
  MSF_INFO_STREAM_COND(!use_fixed_covariance_,
                       "Pose sensor is using covariance "
                       "from sensor");

  MSF_INFO_STREAM_COND(provides_absolute_measurements_,
                       "Pose sensor is handling "
                       "measurements as absolute values");
  MSF_INFO_STREAM_COND(!provides_absolute_measurements_, "Pose sensor is "
                       "handling measurements as relative values");

  ros::NodeHandle nh("msf_updates/" + topic_namespace);
  subPoseWithCovarianceStamped_ =
      nh.subscribe < geometry_msgs::PoseWithCovarianceStamped
          > ("pose_with_covariance_input", 20, &PoseSensorHandler::MeasurementCallback, this);
  subTransformStamped_ = nh.subscribe < geometry_msgs::TransformStamped
      > ("transform_input", 20, &PoseSensorHandler::MeasurementCallback, this);
  subPoseStamped_ = nh.subscribe < geometry_msgs::PoseStamped
      > ("pose_input", 20, &PoseSensorHandler::MeasurementCallback, this);

  z_p_.setZero();
  z_q_.setIdentity();

  if (distortmeas) {
    Eigen::Vector3d meanpos;
    double distortpos_mean;
    pnh.param("distortpos_mean", distortpos_mean, 0.0);
    meanpos.setConstant(distortpos_mean);

    Eigen::Vector3d stddevpos;
    double distortpos_stddev;
    pnh.param("distortpos_stddev", distortpos_stddev, 0.0);
    stddevpos.setConstant(distortpos_stddev);

    Eigen::Vector3d meanatt;
    double distortatt_mean;
    pnh.param("distortatt_mean", distortatt_mean, 0.0);
    meanatt.setConstant(distortatt_mean);

    Eigen::Vector3d stddevatt;
    double distortatt_stddev;
    pnh.param("distortatt_stddev", distortatt_stddev, 0.0);
    stddevatt.setConstant(distortatt_stddev);

    double distortscale_mean;
    pnh.param("distortscale_mean", distortscale_mean, 0.0);
    double distortscale_stddev;
    pnh.param("distortscale_stddev", distortscale_stddev, 0.0);

    distorter_.reset(
        new msf_updates::PoseDistorter(meanpos, stddevpos, meanatt, stddevatt,
                                       distortscale_mean, distortscale_stddev));
  }
}

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void PoseSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::SetNoises(double n_zp,
                                                                  double n_zq) {
  n_zp_ = n_zp;
  n_zq_ = n_zq;
}

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void PoseSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::SetDelay(double delay) {
  delay_ = delay;
}

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void PoseSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::ProcessPoseMeasurement(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr & msg) {
  received_first_measurement_ = true;

  // Get the fixed states.
  int fixedstates = 0;
  static_assert(msf_updates::EKFState::nStateVarsAtCompileTime < 32, "Your state "
      "has more than 32 variables. The code needs to be changed here to have a "
      "larger variable to mark the fixed_states");
  // Do not exceed the 32 bits of int.

  // Get all the fixed states and set flag bits.
  MANAGER_TYPE* mngr = dynamic_cast<MANAGER_TYPE*>(&manager_);

  // TODO(acmarkus): if we have multiple sensor handlers, they all share the same dynparams,
  // which me maybe don't want. E.g. if we have this for multiple AR Markers, we
  // may want to keep one fix --> move this to fixed parameters? Could be handled
  // with parameter namespace then.
  if (mngr) {
    if (mngr->Getcfg().pose_fixed_scale) {
      fixedstates |= 1 << MEASUREMENT_TYPE::AuxState::L;
    }
    if (mngr->Getcfg().pose_fixed_p_ic) {
      fixedstates |= 1 << MEASUREMENT_TYPE::AuxState::p_ic;
    }
    if (mngr->Getcfg().pose_fixed_q_ic) {
      fixedstates |= 1 << MEASUREMENT_TYPE::AuxState::q_ic;
    }
    if (mngr->Getcfg().pose_fixed_p_wv) {
      fixedstates |= 1 << MEASUREMENT_TYPE::AuxState::p_wv;
    }
    if (mngr->Getcfg().pose_fixed_q_wv) {
      fixedstates |= 1 << MEASUREMENT_TYPE::AuxState::q_wv;
    }
  }

  shared_ptr<MEASUREMENT_TYPE> meas(new MEASUREMENT_TYPE(
      n_zp_, n_zq_, measurement_world_sensor_, use_fixed_covariance_,
      provides_absolute_measurements_, this->sensorID,
      enable_mah_outlier_rejection_, mah_threshold_, fixedstates, distorter_));

  meas->MakeFromSensorReading(msg, msg->header.stamp.toSec() - delay_);

  z_p_ = meas->z_p_;  //store this for the init procedure
  z_q_ = meas->z_q_;

  this->manager_.msf_core_->AddMeasurement(meas);
}
template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void PoseSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::MeasurementCallback(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr & msg) {

  this->SequenceWatchDog(msg->header.seq,
                         subPoseWithCovarianceStamped_.getTopic());
  MSF_INFO_STREAM_ONCE(
      "*** pose sensor got first measurement from topic "
          << this->topic_namespace_ << "/"
          << subPoseWithCovarianceStamped_.getTopic() << " ***");
  ProcessPoseMeasurement(msg);
}

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void PoseSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::MeasurementCallback(
    const geometry_msgs::TransformStampedConstPtr & msg) {
  this->SequenceWatchDog(msg->header.seq, subTransformStamped_.getTopic());
  MSF_INFO_STREAM_ONCE(
      "*** pose sensor got first measurement from topic "
          << this->topic_namespace_ << "/" << subTransformStamped_.getTopic()
          << " ***");

  double time_now = msg->header.stamp.toSec();
  const double epsilon = 0.001; // Small time correction to avoid rounding errors in the timestamps.
  if (time_now - timestamp_previous_pose_ <= pose_measurement_minimum_dt_ - epsilon) {
    MSF_WARN_STREAM_THROTTLE(30, "Pose measurement throttling is on, dropping messages"
                             "to be below " +
                             std::to_string(1/pose_measurement_minimum_dt_) + " Hz");
    return;
  }

  timestamp_previous_pose_ = time_now;

  geometry_msgs::PoseWithCovarianceStampedPtr pose(
      new geometry_msgs::PoseWithCovarianceStamped());

  if (!use_fixed_covariance_)  // Take covariance from sensor.
  {
    MSF_WARN_STREAM_THROTTLE(
        2,
        "Provided message type without covariance but set fixed_covariance == "
        "false at the same time. Discarding message.");
    return;
  }

  // Fixed covariance will be set in measurement class -> MakeFromSensorReadingImpl.
  pose->header = msg->header;

  pose->pose.pose.position.x = msg->transform.translation.x;
  pose->pose.pose.position.y = msg->transform.translation.y;
  pose->pose.pose.position.z = msg->transform.translation.z;

  pose->pose.pose.orientation.w = msg->transform.rotation.w;
  pose->pose.pose.orientation.x = msg->transform.rotation.x;
  pose->pose.pose.orientation.y = msg->transform.rotation.y;
  pose->pose.pose.orientation.z = msg->transform.rotation.z;

  ProcessPoseMeasurement(pose);
}

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void PoseSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::MeasurementCallback(
    const geometry_msgs::PoseStampedConstPtr & msg) {
  this->SequenceWatchDog(msg->header.seq, subPoseStamped_.getTopic());
  MSF_INFO_STREAM_ONCE(
      "*** pose sensor got first measurement from topic "
          << this->topic_namespace_ << "/" << subPoseStamped_.getTopic()
          << " ***");

  geometry_msgs::PoseWithCovarianceStampedPtr pose(
      new geometry_msgs::PoseWithCovarianceStamped());

  if (!use_fixed_covariance_)  // Take covariance from sensor.
  {
    MSF_WARN_STREAM_THROTTLE(
        2,
        "Provided message type without covariance but set fixed_covariance =="
        "false at the same time. Discarding message.");
    return;
  }

  // Fixed covariance will be set in measurement class -> MakeFromSensorReadingImpl.

  pose->header = msg->header;

  pose->pose.pose = msg->pose;

  ProcessPoseMeasurement(pose);
}
}  // namespace msf_pose_sensor
#endif  // POSE_SENSORHANDLER_HPP_
