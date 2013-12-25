/*
 * Copyright (C) 2012-2013 Simon Lynen, ASL, ETH Zurich, Switzerland
 * You can contact the author at <slynen at ethz dot ch>
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
#ifndef MSF_STATEDEF_HPP_
#define MSF_STATEDEF_HPP_

#include <Eigen/Dense>
#include <msf_core/msf_fwds.h>
#include <boost/fusion/container.hpp>

namespace msf_updates {

/*
 * This file contains the state definition of the EKF as defined for a given set
 * of sensors / states to estimate.
 */

enum StateDefinition {  // Must not manually set the enum values!
  p,
  v,
  q,
  b_w,
  b_a,
  p_ip
};

namespace {

/***
 * Setup core state, then auxiliary state.
 */
typedef boost::fusion::vector<
    // States varying during propagation - must not change the ordering here for
    // now, CalcQ has the ordering hardcoded.
    msf_core::StateVar_T<Eigen::Matrix<double, 3, 1>, p,
        msf_core::CoreStateWithPropagation>,  ///< Translation from the world frame to the IMU frame expressed in the world frame.
    msf_core::StateVar_T<Eigen::Matrix<double, 3, 1>, v,
        msf_core::CoreStateWithPropagation>,  ///< Velocity of the IMU frame expressed in the world frame.
    msf_core::StateVar_T<Eigen::Quaternion<double>, q,
        msf_core::CoreStateWithPropagation>,  ///< Rotation from the world frame to the IMU frame expressed in the world frame.
    msf_core::StateVar_T<Eigen::Matrix<double, 3, 1>, b_w,
        msf_core::CoreStateWithoutPropagation>,  ///< Gyro biases.                      
    msf_core::StateVar_T<Eigen::Matrix<double, 3, 1>, b_a,
        msf_core::CoreStateWithoutPropagation>,  ///< Acceleration biases.              

    // States not varying during propagation.
    msf_core::StateVar_T<Eigen::Matrix<double, 3, 1>, p_ip>  ///< Translation from the IMU frame to the position sensor frame expressed in the IMU frame.
> fullState_T;
}

typedef msf_core::GenericState_T<fullState_T, StateDefinition> EKFState;  ///< The state we want to use in this EKF.
typedef shared_ptr<EKFState> EKFStatePtr;
typedef shared_ptr<const EKFState> EKFStateConstPtr;
}
#include <msf_updates/static_ordering_assertions.h>  // DO NOT REMOVE THIS
#endif  // MSF_STATEDEF_HPP_
