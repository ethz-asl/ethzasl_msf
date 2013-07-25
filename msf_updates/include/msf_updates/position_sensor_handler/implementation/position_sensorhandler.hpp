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

#include <msf_core/msf_types.tpp>
#include <msf_core/eigen_utils.h>
#include <msf_core/gps_conversion.h>

namespace msf_position_sensor{
template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
PositionSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::PositionSensorHandler(MANAGER_TYPE& meas, std::string topic_namespace, std::string parameternamespace) :
SensorHandler<msf_updates::EKFState>(meas, topic_namespace, parameternamespace), n_zp_(1e-6), delay_(0)
{
  ros::NodeHandle pnh("~/position_sensor");
  pnh.param("position_use_fixed_covariance", use_fixed_covariance_, false);
  pnh.param("position_absolute_measurements", provides_absolute_measurements_, false);

  MSF_INFO_STREAM_COND(use_fixed_covariance_, "Position sensor is using fixed covariance");
  MSF_INFO_STREAM_COND(!use_fixed_covariance_, "Position sensor is using covariance from sensor");

  MSF_INFO_STREAM_COND(provides_absolute_measurements_, "Position sensor is handling measurements as absolute values");
  MSF_INFO_STREAM_COND(!provides_absolute_measurements_, "Position sensor is handling measurements as relative values");

  ros::NodeHandle nh("msf_updates");

  subPointStamped_ = nh.subscribe<geometry_msgs::PointStamped>("position_input", 20, &PositionSensorHandler::measurementCallback, this);
  subTransformStamped_ = nh.subscribe<geometry_msgs::TransformStamped>("transform_input", 20, &PositionSensorHandler::measurementCallback, this);
  subNavSatFix_ = nh.subscribe<sensor_msgs::NavSatFix>("navsatfix_input", 20, &PositionSensorHandler::measurementCallback, this);

  z_p_.setZero();

}

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void PositionSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::setNoises(double n_zp)
{
  n_zp_ = n_zp;
}

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void PositionSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::setDelay(double delay)
{
  delay_ = delay;
}

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void PositionSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::adjustGPSZReference(double current_z)
{
  gpsConversion_.adjustReference(z_p_(2) - current_z);
}

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void PositionSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::processPositionMeasurement(const msf_updates::PointWithCovarianceStampedConstPtr& msg){
  //get the fixed states
  int fixedstates = 0;
  static_assert(msf_updates::EKFState::nStateVarsAtCompileTime < 32, "Your state has more than 32 variables. "
                          "The code needs to be changed here to have a larger variable to mark the fixed_states"); //do not exceed the 32 bits of int


  if (!use_fixed_covariance_ && msg->covariance[0] == 0)  // take covariance from sensor
  {
    MSF_WARN_STREAM_THROTTLE(2,"Provided message type without covariance but set fixed_covariance=false at the same time. Discarding message.");
    return;
  }

  //get all the fixed states and set flag bits
  MANAGER_TYPE* mngr = dynamic_cast<MANAGER_TYPE*>(&manager_);

  if(mngr){
    if (mngr->getcfg().position_fixed_p_ip){
      fixedstates |= 1 << msf_updates::EKFState::StateDefinition_T::p_ip;
    }
  }

  shared_ptr<MEASUREMENT_TYPE> meas( new MEASUREMENT_TYPE(n_zp_, use_fixed_covariance_, provides_absolute_measurements_, this->sensorID, fixedstates));

  meas->makeFromSensorReading(msg, msg->header.stamp.toSec() - delay_);

  z_p_ = meas->z_p_; //store this for the init procedure

  this->manager_.msf_core_->addMeasurement(meas);
                                                                                       }

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void PositionSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::measurementCallback(const geometry_msgs::PointStampedConstPtr & msg)
{
  this->sequenceWatchDog(msg->header.seq, subPointStamped_.getTopic());

  MSF_INFO_STREAM_ONCE("*** position sensor got first measurement from topic "<<this->topic_namespace_<<"/"<<subPointStamped_.getTopic()<<" ***");

  msf_updates::PointWithCovarianceStampedPtr pointwCov(new msf_updates::PointWithCovarianceStamped);
  pointwCov->header = msg->header;
  pointwCov->point = msg->point;

  processPositionMeasurement(pointwCov);
}

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void PositionSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::measurementCallback(const geometry_msgs::TransformStampedConstPtr & msg)
{
  this->sequenceWatchDog(msg->header.seq, subTransformStamped_.getTopic());

  MSF_INFO_STREAM_ONCE("*** position sensor got first measurement from topic "<<this->topic_namespace_<<"/"<<subTransformStamped_.getTopic()<<" ***");

  if(msg->header.seq%5!=0){ //slow down vicon
    MSF_WARN_STREAM_ONCE("Measurement throttling is on, dropping every but the 5th message");
    return;
  }

  msf_updates::PointWithCovarianceStampedPtr pointwCov(new msf_updates::PointWithCovarianceStamped);
  pointwCov->header = msg->header;

  //fixed covariance will be set in measurement class -> makeFromSensorReadingImpl

  pointwCov->point.x = msg->transform.translation.x;
  pointwCov->point.y = msg->transform.translation.y;
  pointwCov->point.z = msg->transform.translation.z;

  processPositionMeasurement(pointwCov);
}


template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void PositionSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::measurementCallback(const sensor_msgs::NavSatFixConstPtr& msg)
{
  this->sequenceWatchDog(msg->header.seq, subNavSatFix_.getTopic());

  MSF_INFO_STREAM_ONCE("*** position sensor got first measurement from topic "<<this->topic_namespace_<<"/"<<subNavSatFix_.getTopic()<<" ***");

  //fixed covariance will be set in measurement class -> makeFromSensorReadingImpl

  static bool referenceinit = false; //TODO dynreconf reset ref
  if(!referenceinit){
    gpsConversion_.initReference(msg->latitude, msg->longitude, msg->altitude);
    MSF_WARN_STREAM("Initialized GPS reference of topic: "<<this->topic_namespace_<<"/"<<subNavSatFix_.getTopic()<<
                    " to lat/lon/alt: ["<<msg->latitude<<", "<<msg->longitude<<", "<<msg->altitude<<"]");
    referenceinit = true;
  }

  msf_core::Vector3 ecef = gpsConversion_.wgs84ToEcef(msg->latitude, msg->longitude, msg->altitude);
  msf_core::Vector3 enu = gpsConversion_.ecefToEnu(ecef);

  msf_updates::PointWithCovarianceStampedPtr pointwCov(new msf_updates::PointWithCovarianceStamped);
  pointwCov->header = msg->header;

  //store the ENU data in the position fields
  pointwCov->point.x = enu[0];
  pointwCov->point.y = enu[1];
  pointwCov->point.z = enu[2];

  //get the covariance TODO: handle the cases differently?
  if(msg->position_covariance_type == sensor_msgs::NavSatFix::COVARIANCE_TYPE_KNOWN){
    pointwCov->covariance = msg->position_covariance;
  }else if(msg->position_covariance_type == sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN){
    pointwCov->covariance = msg->position_covariance;
  }else if(msg->position_covariance_type == sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED){ //from DOP
    pointwCov->covariance = msg->position_covariance;
  }

  processPositionMeasurement(pointwCov);
}

}
