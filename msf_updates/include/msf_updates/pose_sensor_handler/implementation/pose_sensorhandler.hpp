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

#include <msf_core/eigen_utils.h>

namespace msf_pose_sensor{
template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
PoseSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::PoseSensorHandler(MANAGER_TYPE& meas, std::string topic_namespace, std::string parameternamespace, bool distortmeas) :
SensorHandler<msf_updates::EKFState>(meas, topic_namespace, parameternamespace), n_zp_(1e-6), n_zq_(1e-6), delay_(0)
{
  ros::NodeHandle pnh("~/"+parameternamespace);

  ROS_INFO_STREAM("Loading parameters for pose sensor from namespace: "<<pnh.getNamespace());

  pnh.param("pose_absolute_measurements", provides_absolute_measurements_, true);
  pnh.param("pose_measurement_world_sensor", measurement_world_sensor_, true);
  pnh.param("pose_use_fixed_covariance", use_fixed_covariance_, false);

  ROS_INFO_COND(measurement_world_sensor_, "Pose sensor is interpreting measurement as sensor w.r.t. world");
  ROS_INFO_COND(!measurement_world_sensor_, "Pose sensor is interpreting measurement as world w.r.t. sensor (e.g. ethzasl_ptam)");

  ROS_INFO_COND(use_fixed_covariance_, "Pose sensor is using fixed covariance");
  ROS_INFO_COND(!use_fixed_covariance_, "Pose sensor is using covariance from sensor");

  ROS_INFO_COND(provides_absolute_measurements_, "Pose sensor is handling measurements as absolute values");
  ROS_INFO_COND(!provides_absolute_measurements_, "Pose sensor is handling measurements as relative values");

  ros::NodeHandle nh("msf_updates/" + topic_namespace);
  subPoseWithCovarianceStamped_ = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("pose_with_covariance_input", 20, &PoseSensorHandler::measurementCallback, this);
  subTransformStamped_ = nh.subscribe<geometry_msgs::TransformStamped>("transform_input", 20, &PoseSensorHandler::measurementCallback, this);
  subPoseStamped_ = nh.subscribe<geometry_msgs::PoseStamped>("pose_input", 20, &PoseSensorHandler::measurementCallback, this);

  z_p_.setZero();
  z_q_.setIdentity();

  if(distortmeas){
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

    distorter_.reset(new msf_updates::PoseDistorter(meanpos, stddevpos, meanatt, stddevatt, distortscale_mean, distortscale_stddev));
  }
}

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void PoseSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::setNoises(double n_zp, double n_zq)
{
  n_zp_ = n_zp;
  n_zq_ = n_zq;
}

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void PoseSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::setDelay(double delay)
{
  delay_ = delay;
}


template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void PoseSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::ProcessPoseMeasurement(const geometry_msgs::PoseWithCovarianceStampedConstPtr & msg)
{

  //get the fixed states
  int fixedstates = 0;
  BOOST_STATIC_ASSERT_MSG(msf_updates::EKFState::nStateVarsAtCompileTime < 32, "Your state has more than 32 variables. "
                          "The code needs to be changed here to have a larger variable to mark the fixed_states"); //do not exceed the 32 bits of int

  //get all the fixed states and set flag bits
  MANAGER_TYPE* mngr = dynamic_cast<MANAGER_TYPE*>(&manager_);

  if(mngr){
    if (mngr->getcfg().pose_fixed_scale){
      fixedstates |= 1 << msf_updates::EKFState::StateDefinition_T::L;
    }
    if (mngr->getcfg().pose_fixed_p_ic){
      fixedstates |= 1 << msf_updates::EKFState::StateDefinition_T::p_ic;
    }
    if (mngr->getcfg().pose_fixed_q_ic){
      fixedstates |= 1 << msf_updates::EKFState::StateDefinition_T::q_ic;
    }
    if (mngr->getcfg().pose_fixed_p_wv){
      fixedstates |= 1 << msf_updates::EKFState::StateDefinition_T::p_wv;
    }
    if (mngr->getcfg().pose_fixed_q_wv){
      fixedstates |= 1 << msf_updates::EKFState::StateDefinition_T::q_wv;
    }
  }

  std::shared_ptr<MEASUREMENT_TYPE> meas( new MEASUREMENT_TYPE(n_zp_, n_zq_, measurement_world_sensor_, use_fixed_covariance_, provides_absolute_measurements_, this->sensorID, fixedstates, distorter_));

  meas->makeFromSensorReading(msg, msg->header.stamp.toSec() - delay_);

  z_p_ = meas->z_p_; //store this for the init procedure
  z_q_ = meas->z_q_;

  this->manager_.msf_core_->addMeasurement(meas);
}
template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void PoseSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::measurementCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr & msg)
{

  this->sequenceWatchDog(msg->header.seq, subPoseWithCovarianceStamped_.getTopic());
  ROS_INFO_STREAM_ONCE("*** pose sensor got first measurement from topic "<<this->topic_namespace_<<"/"<<subPoseWithCovarianceStamped_.getTopic()<<" ***");
  ProcessPoseMeasurement(msg);

}

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void PoseSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::measurementCallback(const geometry_msgs::TransformStampedConstPtr & msg)
{
  this->sequenceWatchDog(msg->header.seq, subTransformStamped_.getTopic());
  ROS_INFO_STREAM_ONCE("*** pose sensor got first measurement from topic "<<this->topic_namespace_<<"/"<<subTransformStamped_.getTopic()<<" ***");

  if(msg->header.seq%5!=0){ //slow down vicon
    ROS_WARN_STREAM_THROTTLE(30, "Measurement throttling is on, dropping every but the 5th message");
    return;
  }

  geometry_msgs::PoseWithCovarianceStampedPtr pose(new geometry_msgs::PoseWithCovarianceStamped());

  if (!use_fixed_covariance_)  // take covariance from sensor
  {
    ROS_WARN_STREAM_THROTTLE(2,"Provided message type without covariance but set fixed_covariance=false at the same time. Discarding message.");
    return;
  }

  //fixed covariance will be set in measurement class -> makeFromSensorReadingImpl

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
void PoseSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::measurementCallback(const geometry_msgs::PoseStampedConstPtr & msg)
{
  this->sequenceWatchDog(msg->header.seq, subPoseStamped_.getTopic());
  ROS_INFO_STREAM_ONCE("*** pose sensor got first measurement from topic "<<this->topic_namespace_<<"/"<<subPoseStamped_.getTopic()<<" ***");

  geometry_msgs::PoseWithCovarianceStampedPtr pose(new geometry_msgs::PoseWithCovarianceStamped());

  if (!use_fixed_covariance_)  // take covariance from sensor
  {
    ROS_WARN_STREAM_THROTTLE(2,"Provided message type without covariance but set fixed_covariance=false at the same time. Discarding message.");
    return;
  }

  //fixed covariance will be set in measurement class -> makeFromSensorReadingImpl

  pose->header = msg->header;

  pose->pose.pose = msg->pose;

  ProcessPoseMeasurement(pose);
}

}
