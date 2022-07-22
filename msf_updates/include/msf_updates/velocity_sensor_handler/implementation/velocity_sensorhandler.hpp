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
#ifndef VELOCITY_SENSORHANDLER_HPP_
#define VELOCITY_SENSORHANDLER_HPP_
#include <msf_core/eigen_utils.h>
#include <msf_core/msf_types.h>

namespace msf_velocity_sensor {
template <typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
VelocityXYSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::
    VelocityXYSensorHandler(MANAGER_TYPE& meas, std::string topic_namespace,
                            std::string parameternamespace)
    : SensorHandler<msf_updates::EKFState>(meas, topic_namespace,
                                           parameternamespace),
      n_zv_(1e-6),  // TODO(clanegge): What should the value of this be?
      delay_(0.0) {
  // TODO:
}

template <typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void VelocityXYSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::SetNoises(
    double n_zv) {
  n_zv_ = n_zv;
}

template <typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void VelocityXYSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::SetDelay(
    double delay) {
  delay_ = delay;
}

template <typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void VelocityXYSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::
    ProcessVelocityMeasurement(
        const geometry_msgs::TwistWithCovarianceStampedConstPtr& msg) {
  received_first_measurement_ = true;

  // TODO(clanegge): process measurements and add measurement to core
}

template <typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void VelocityXYSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::
    MeasurementCallback(
        const geometry_msgs::TwistWithCovarianceStampedConstPtr& msg) {
  this->SequenceWatchDog(msg->header.seq,
                         subTwistWCovarianceStamped_.getTopic());
  MSF_INFO_STREAM_ONCE("*** velocity sensor got first measurement from topic "
                       << this->topic_namespace_ << "/"
                       << subTransformStamped_.getTopic() << " ***");

  ProcessVelocityMeasurement(msg);
}

}  // namespace msf_velocity_sensor

#endif  // VELOCITY_SENSORHANDLER_HPP_