/*
 * msf_statedef.hpp
 *
 *  Created on: Nov 6, 2012
 *      Author: slynen
 */

#ifndef MSF_STATEDEF_HPP_
#define MSF_STATEDEF_HPP_

#include <Eigen/Dense>
#include <msf_core/msf_fwd.hpp>

namespace msf_core{

enum{ //must not manually set the enum values!
	p_,
	v_,
	q_,
	b_w_,
	b_a_,
	L_,
	q_wv_,
	q_ci_,
	p_ci_
};

namespace{

//setup core state, then auxiliary state
typedef boost::fusion::vector<
		// states varying during propagation
		StateVar_T<Eigen::Matrix<double, 3, 1>, p_>,         ///< position (IMU centered)          (0-2 / 0-2)
		StateVar_T<Eigen::Matrix<double, 3, 1>, v_>,         ///< velocity                         (3- 5 / 3- 5)
		StateVar_T<Eigen::Quaternion<double>, q_>,           ///< attitude                         (6- 9 / 6- 8)
		StateVar_T<Eigen::Matrix<double, 3, 1>, b_w_>,       ///< gyro biases                      (10-12 / 9-11)
		StateVar_T<Eigen::Matrix<double, 3, 1>, b_a_>,       ///< acceleration biases              (13-15 / 12-14)

		// states not varying during propagation
		StateVar_T<Eigen::Matrix<double, 1, 1>, L_>,                              ///< visual scale                     (16 / 15)
		StateVar_T<Eigen::Quaternion<double>, q_wv_>,        ///< vision-world attitude drift      (17-20 / 16-18)
		StateVar_T<Eigen::Quaternion<double>, q_ci_>,        ///< camera-imu attitude calibration  (21-24 / 19-21)
		StateVar_T<Eigen::Matrix<double, 3, 1>, p_ci_>      ///< camera-imu position calibration  (25-27 / 22-24)
> fullState_T;
}

typedef EKFState_T<msf_core::fullState_T> EKFState;

}


#endif /* MSF_STATEDEF_HPP_ */
