/*

Copyright (c) 2012, Simon Lynen, ASL, ETH Zurich, Switzerland
You can contact the author at <slynen at ethz dot ch>

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


#ifndef MSF_STATEDEF_HPP_
#define MSF_STATEDEF_HPP_

#include <Eigen/Dense>
#include <msf_core/msf_fwds.h>
#include <boost/fusion/container.hpp>
#include <boost/static_assert.hpp>

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
  p_pi
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
    msf_core::StateVar_T<Eigen::Matrix<double, 3, 1>, p_pi>                                                            ///< prism-imu position calibration
> fullState_T;
}

typedef msf_core::GenericState_T<fullState_T, StateDefinition> EKFState; ///<the state we want to use in this EKF
typedef boost::shared_ptr<EKFState> EKFStatePtr;
typedef boost::shared_ptr<const EKFState> EKFStateConstPtr;

}

#include <msf_updates/static_ordering_assertions.h> //DO NOT REMOVE THIS

#endif /* MSF_STATEDEF_HPP_ */
