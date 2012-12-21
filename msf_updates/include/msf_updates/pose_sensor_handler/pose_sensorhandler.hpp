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
PoseSensorHandler::PoseSensorHandler(msf_core::MSF_SensorManager<msf_updates::EKFState>& meas) :
        SensorHandler<msf_updates::EKFState>(meas), n_zp_(1e-6), n_zq_(1e-6), delay_(0)
{
  ros::NodeHandle pnh("~");
  pnh.param("measurement_world_sensor", measurement_world_sensor_, true);
  pnh.param("use_fixed_covariance", use_fixed_covariance_, false);

  ROS_INFO_COND(measurement_world_sensor_, "interpreting measurement as sensor w.r.t. world");
  ROS_INFO_COND(!measurement_world_sensor_, "interpreting measurement as world w.r.t. sensor (e.g. ethzasl_ptam)");

  ROS_INFO_COND(use_fixed_covariance_, "using fixed covariance");
  ROS_INFO_COND(!use_fixed_covariance_, "using covariance from sensor");


  ros::NodeHandle nh("msf_updates");
  subPoseWithCovarianceStamped_ = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("pose_with_covariance_input", 1, &PoseSensorHandler::measurementCallback, this);
  subTransformStamped_ = nh.subscribe<geometry_msgs::TransformStamped>("transform_input", 1, &PoseSensorHandler::measurementCallback, this);
  subPoseStamped_ = nh.subscribe<geometry_msgs::PoseStamped>("pose_input", 1, &PoseSensorHandler::measurementCallback, this);
}

void PoseSensorHandler::setNoises(double n_zp, double n_zq)
{
  n_zp_ = n_zp;
  n_zq_ = n_zq;
}

void PoseSensorHandler::setDelay(double delay)
{
  delay_ = delay;
}
void PoseSensorHandler::measurementCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr & msg)
{


  boost::shared_ptr<pose_measurement::PoseMeasurement> meas( new pose_measurement::PoseMeasurement(n_zp_, n_zq_, measurement_world_sensor_, use_fixed_covariance_));
  meas->makeFromSensorReading(msg, msg->header.stamp.toSec() - delay_);

  z_p_ = meas->z_p_; //store this for the init procedure
  z_q_ = meas->z_q_;

  this->manager_.msf_core_->addMeasurement(meas);
}

void PoseSensorHandler::measurementCallback(const geometry_msgs::TransformStampedConstPtr & msg)
{

  if(msg->header.seq%5!=0){ //slow down vicon
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

  measurementCallback(pose);
}

void PoseSensorHandler::measurementCallback(const geometry_msgs::PoseStampedConstPtr & msg)
{

  geometry_msgs::PoseWithCovarianceStampedPtr pose(new geometry_msgs::PoseWithCovarianceStamped());

  if (!use_fixed_covariance_)  // take covariance from sensor
  {
    ROS_WARN_STREAM_THROTTLE(2,"Provided message type without covariance but set fixed_covariance=false at the same time. Discarding message.");
    return;
  }

  //fixed covariance will be set in measurement class -> makeFromSensorReadingImpl

  pose->header = msg->header;

  pose->pose.pose = msg->pose;

  measurementCallback(pose);
}

}
