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
 

namespace msf_updates{

/*
 * This file contains the state definition of the EKF as defined for a given set of sensors / states to estimate
 */

enum StateDefinition{ //must not manually set the enum values!
  p,
  v,
  q,
  b_w,
  b_a,
  L,
  q_wv,
  p_wv,
  q_ic,
  p_ic,
  p_ip
};

namespace{


/***
 * setup core state, then auxiliary state
 */
typedef boost::fusion::vector<
    // states varying during propagation - must not change the ordering here for now, CalcQ has the ordering hardcoded
    msf_core::StateVar_T<Eigen::Matrix<double, 3, 1>, p, msf_core::CoreStateWithPropagation>,                          ///< position (IMU centered)          (0-2 / 0-2)
    msf_core::StateVar_T<Eigen::Matrix<double, 3, 1>, v, msf_core::CoreStateWithPropagation>,                          ///< velocity                         (3- 5 / 3- 5)
    msf_core::StateVar_T<Eigen::Quaternion<double>, q, msf_core::CoreStateWithPropagation>,                            ///< attitude                         (6- 9 / 6- 8)
    msf_core::StateVar_T<Eigen::Matrix<double, 3, 1>, b_w, msf_core::CoreStateWithoutPropagation>,                     ///< gyro biases                      (10-12 / 9-11)
    msf_core::StateVar_T<Eigen::Matrix<double, 3, 1>, b_a, msf_core::CoreStateWithoutPropagation>,                     ///< acceleration biases              (13-15 / 12-14)

    // states not varying during propagation
    msf_core::StateVar_T<Eigen::Matrix<double, 1, 1>, L>,                                                              ///< visual scale
    msf_core::StateVar_T<Eigen::Quaternion<double>, q_wv, msf_core::AuxiliaryNonTemporalDrifting>,                     ///< vision-world attitude drift
    msf_core::StateVar_T<Eigen::Matrix<double, 3, 1>, p_wv>,                                                           ///< vision world position drift
    msf_core::StateVar_T<Eigen::Quaternion<double>, q_ic>,                                                             ///< camera-imu attitude calibration
    msf_core::StateVar_T<Eigen::Matrix<double, 3, 1>, p_ic>,                                                           ///< camera-imu position calibration
    msf_core::StateVar_T<Eigen::Matrix<double, 3, 1>, p_ip>
> fullState_T;
}

typedef msf_core::GenericState_T<fullState_T, StateDefinition> EKFState; ///<the state we want to use in this EKF
typedef shared_ptr<EKFState> EKFStatePtr;
typedef shared_ptr<const EKFState> EKFStateConstPtr;

}

#include <msf_updates/static_ordering_assertions.h> //DO NOT REMOVE THIS

#endif /* MSF_STATEDEF_HPP_ */
