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
#include <msf_core/gps_conversion.h>

namespace msf_spherical_position
{
template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
  AngleSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::AngleSensorHandler(MANAGER_TYPE& meas,
                                                                         std::string topic_namespace,
                                                                         std::string parameternamespace) :
      SensorHandler<msf_updates::EKFState>(meas, topic_namespace, parameternamespace), n_za_(1e-6), delay_(0)
  {
    ros::NodeHandle pnh("~/spherical_position_sensor");
    pnh.param("use_fixed_covariance", use_fixed_covariance_, false);
    pnh.param("absolute_measurements", provides_absolute_measurements_, false);

    ROS_INFO_COND(use_fixed_covariance_, "Angle sensor is using fixed covariance");
    ROS_INFO_COND(!use_fixed_covariance_, "Angle sensor is using covariance from sensor");

    ROS_INFO_COND(provides_absolute_measurements_, "Angle sensor is handling measurements as absolute values");
    ROS_INFO_COND(!provides_absolute_measurements_, "Angle sensor is handling measurements as relative values");

    ros::NodeHandle nh("msf_updates");

    subPointStamped_ = nh.subscribe<geometry_msgs::PointStamped>("angle_input", 20,
                                                                 &AngleSensorHandler::measurementCallback, this);

    z_a_.setZero();

  }

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
  void AngleSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::setNoises(double n_za)
  {
    n_za_ = n_za;
  }

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
  void AngleSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::setDelay(double delay)
  {
    delay_ = delay;
  }

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
  void AngleSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::measurementCallback(
      const geometry_msgs::PointStampedConstPtr & msg)
  {
    this->sequenceWatchDog(msg->header.seq, subPointStamped_.getTopic());

    ROS_INFO_STREAM_ONCE(
        "*** position sensor got first measurement from topic " << this->topic_namespace_ << "/"
            << subPointStamped_.getTopic() << " ***");

    //get the fixed states
    int fixedstates = 0;
    BOOST_STATIC_ASSERT_MSG(msf_updates::EKFState::nStateVarsAtCompileTime < 32,
                            "Your state has more than 32 variables. "
                            "The code needs to be changed here to have a larger variable to mark the fixed_states"); //do not exceed the 32 bits of int

    if (!use_fixed_covariance_ /*&& msg->covariance[0] == 0*/)  // take covariance from sensor
    {
      ROS_WARN_STREAM_THROTTLE(
          2,
          "Provided message type without covariance but set fixed_covariance=false at the same time. Discarding message.");
      return;
    }

    //get all the fixed states and set flag bits
    MANAGER_TYPE* mngr = dynamic_cast<MANAGER_TYPE*>(&manager_);

    if (mngr)
    {
      if (mngr->getcfg().fixed_p_ip)
      {
        fixedstates |= 1 << msf_updates::EKFState::StateDefinition_T::p_ip;
      }
    }

    typename boost::shared_ptr<MEASUREMENT_TYPE> meas(
        new MEASUREMENT_TYPE(n_za_, use_fixed_covariance_, provides_absolute_measurements_, this->sensorID,
                             fixedstates));

    meas->makeFromSensorReading(msg, msg->header.stamp.toSec() - delay_);

    z_a_ = meas->z_a_; //store this for the init procedure

    this->manager_.msf_core_->addMeasurement(meas);
  }

// ### distance sensor implementation

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
  DistanceSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::DistanceSensorHandler(MANAGER_TYPE& meas,
                                                                               std::string topic_namespace,
                                                                               std::string parameternamespace) :
      SensorHandler<msf_updates::EKFState>(meas, topic_namespace, parameternamespace), n_zd_(1e-6), delay_(0)
  {
    ros::NodeHandle pnh("~/spherical_position_sensor");
    pnh.param("use_fixed_covariance", use_fixed_covariance_, false);
    pnh.param("absolute_measurements", provides_absolute_measurements_, false);

    ROS_INFO_COND(use_fixed_covariance_, "Distance sensor is using fixed covariance");
    ROS_INFO_COND(!use_fixed_covariance_, "Distance sensor is using covariance from sensor");

    ROS_INFO_COND(provides_absolute_measurements_, "Distance sensor is handling measurements as absolute values");
    ROS_INFO_COND(!provides_absolute_measurements_, "Distance sensor is handling measurements as relative values");

    ros::NodeHandle nh("msf_updates");

    subPointStamped_ = nh.subscribe<geometry_msgs::PointStamped>("distance_input", 20,
                                                                 &DistanceSensorHandler::measurementCallback, this);

    z_d_.setZero();

  }

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
  void DistanceSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::setNoises(double n_za)
  {
    n_zd_ = n_za;
  }

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
  void DistanceSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::setDelay(double delay)
  {
    delay_ = delay;
  }

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
  void DistanceSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::measurementCallback(
      const geometry_msgs::PointStampedConstPtr & msg)
  {
    this->sequenceWatchDog(msg->header.seq, subPointStamped_.getTopic());

    ROS_INFO_STREAM_ONCE(
        "*** position sensor got first measurement from topic " << this->topic_namespace_ << "/"
            << subPointStamped_.getTopic() << " ***");

    //get the fixed states
    int fixedstates = 0;
    BOOST_STATIC_ASSERT_MSG(msf_updates::EKFState::nStateVarsAtCompileTime < 32,
                            "Your state has more than 32 variables. "
                            "The code needs to be changed here to have a larger variable to mark the fixed_states"); //do not exceed the 32 bits of int

    if (!use_fixed_covariance_ /*&& msg->covariance[0] == 0*/)  // take covariance from sensor
    {
      ROS_WARN_STREAM_THROTTLE(
          2,
          "Provided message type without covariance but set fixed_covariance=false at the same time. Discarding message.");
      return;
    }

    //get all the fixed states and set flag bits
    MANAGER_TYPE* mngr = dynamic_cast<MANAGER_TYPE*>(&manager_);

    if (mngr)
    {
      if (mngr->getcfg().fixed_p_ip)
      {
        fixedstates |= 1 << msf_updates::EKFState::StateDefinition_T::p_ip;
      }
    }

    typename boost::shared_ptr<MEASUREMENT_TYPE> meas(
        new MEASUREMENT_TYPE(n_zd_, use_fixed_covariance_, provides_absolute_measurements_, this->sensorID,
                             fixedstates));

    meas->makeFromSensorReading(msg, msg->header.stamp.toSec() - delay_);

    z_d_ = meas->z_d_; //store this for the init procedure

    this->manager_.msf_core_->addMeasurement(meas);
  }

}
