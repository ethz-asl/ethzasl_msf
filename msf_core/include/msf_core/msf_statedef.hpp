/*
 * msf_statedef.hpp
 *
 *  Created on: Nov 6, 2012
 *      Author: slynen
 */

#ifndef MSF_STATEDEF_HPP_
#define MSF_STATEDEF_HPP_

namespace msf_core{
namespace{


enum{ //must not manually set the enum values!
	p_ci,
	q_ci,
	q_wv,
	L
};

//setup core state, then auxiliary state
typedef boost::fusion::vector<
		StateVar_T<Eigen::Matrix<double, 3, 1>, p_ci >,
		StateVar_T<Eigen::Quaterniond, q_ci >,
		StateVar_T<Eigen::Quaterniond, q_wv >,
		StateVar_T<Eigen::Matrix<double, 1, 1>, L >
> fullState_T;
}

typedef EKFState_T<msf_core::fullState_T> EKFState;

}


#endif /* MSF_STATEDEF_HPP_ */
