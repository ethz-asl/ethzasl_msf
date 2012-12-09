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
#include <msf_core/msf_fwds.hpp>
#include <boost/fusion/container.hpp>

namespace msf_core{

/*
 * This file contains the state definition of the EKF
 */

enum{ //must not manually set the enum values!
	p_,
	v_,
	q_,
	b_w_,
	b_a_,
	L_,
	q_wv_,
	q_ci_,
	p_ci_/*,
	q_int_*/
};

namespace{


/***
 * setup core state, then auxiliary state
 */
typedef boost::fusion::vector<
		// states varying during propagation - must not change the ordering here for now, CalcQ has the ordering hardcoded
		StateVar_T<Eigen::Matrix<double, 3, 1>, p_, CoreStateWithPropagation>,				///< position (IMU centered)          (0-2 / 0-2)
		StateVar_T<Eigen::Matrix<double, 3, 1>, v_, CoreStateWithPropagation>,				///< velocity                         (3- 5 / 3- 5)
		StateVar_T<Eigen::Quaternion<double>, q_, CoreStateWithPropagation>,				///< attitude                         (6- 9 / 6- 8)
		StateVar_T<Eigen::Matrix<double, 3, 1>, b_w_, CoreStateWithoutPropagation>,			///< gyro biases                      (10-12 / 9-11)
		StateVar_T<Eigen::Matrix<double, 3, 1>, b_a_, CoreStateWithoutPropagation>,			///< acceleration biases              (13-15 / 12-14)

		// states not varying during propagation
		StateVar_T<Eigen::Matrix<double, 1, 1>, L_>,										///< visual scale                     (16 / 15)
		StateVar_T<Eigen::Quaternion<double>, q_wv_, AuxiliaryNonTemporalDrifting>,			///< vision-world attitude drift      (17-20 / 16-18)
		StateVar_T<Eigen::Quaternion<double>, q_ci_>,										///< camera-imu attitude calibration  (21-24 / 19-21)
		StateVar_T<Eigen::Matrix<double, 3, 1>, p_ci_>/*,									///< camera-imu position calibration  (25-27 / 22-24)
		StateVar_T<Eigen::Quaternion<double>, q_int_>	*/									///< this is the integrated ang. vel. no corrections applied, to use for delta rot in external algos...

> fullState_T;
}

typedef GenericState_T<msf_core::fullState_T> EKFState; ///<the state we want to use in this EKF
typedef boost::shared_ptr<EKFState> EKFStatePtr;
typedef boost::shared_ptr<const EKFState> EKFStateConstPtr;

}
#endif /* MSF_STATEDEF_HPP_ */
