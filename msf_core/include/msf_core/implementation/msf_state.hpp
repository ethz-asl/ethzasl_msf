/*
 *  Created on: Nov 7, 2012
 *      Author: slynen
 */

#include <msf_core/msf_types.tpp>
#include <msf_core/msf_tmp.hpp>
#include <msf_core/msf_statedef.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <msf_core/eigen_conversions.h>
#include <sensor_fusion_comm/ExtState.h>
#include <sensor_fusion_comm/DoubleArrayStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <msf_core/msf_userdefinedcalculationbase.hpp>

namespace msf_core{

/// resets the state
/**
 * 3D vectors: 0; quaternion: unit quaternion; scale: 1; time:0; Error covariance: zeros
 */
template<typename stateSequence_T>
void GenericState_T<stateSequence_T>::reset(boost::shared_ptr<UserDefinedCalculationBase> usercalc){

	// reset all states
	boost::fusion::for_each(
			statevars_,
			msf_tmp::resetState()
	);

	//reset system inputs
	w_m_.setZero();
	a_m_.setZero();

	q_int_.setIdentity();

	P_.setZero();
	time_ = 0;

	//now call the user provided function
	usercalc->resetState(*this);
}



/// writes the covariance corresponding to position and attitude to cov
template<typename stateSequence_T>
void GenericState_T<stateSequence_T>::getPoseCovariance(geometry_msgs::PoseWithCovariance::_covariance_type & cov){
	BOOST_STATIC_ASSERT(geometry_msgs::PoseWithCovariance::_covariance_type::static_size == 36);

	typedef typename msf_tmp::getEnumStateType<stateVector_T, msf_core::p_>::value p_type;
	typedef typename msf_tmp::getEnumStateType<stateVector_T, msf_core::q_>::value q_type;

	//get indices of position and attitude in the covariance matrix
	static const int idxstartcorr_p = msf_tmp::getStartIndex<stateVector_T, p_type, msf_tmp::CorrectionStateLengthForType>::value;
	static const int idxstartcorr_q = msf_tmp::getStartIndex<stateVector_T, q_type, msf_tmp::CorrectionStateLengthForType>::value;

	//TODO: remove after initial debugging
	BOOST_STATIC_ASSERT(idxstartcorr_p==0);
	BOOST_STATIC_ASSERT(idxstartcorr_q==6);

	/*        |  cov_p_p  |  cov_p_q  |
	 *        |           |           |
	 * cov =  |-----------|-----------|
	 *        |           |           |
	 *        |  cov_q_p  |  cov_q_q  |
	 */

	for (int i = 0; i < 9; i++)
		cov[i / 3 * 6 + i % 3] = P_((i / 3 + idxstartcorr_p) * nErrorStatesAtCompileTime + i % 3);

	for (int i = 0; i < 9; i++)
		cov[i / 3 * 6 + (i % 3 + 3)] = P_((i / 3 + idxstartcorr_p) * nErrorStatesAtCompileTime + (i % 3 + 6));

	for (int i = 0; i < 9; i++)
		cov[(i / 3 + 3) * 6 + i % 3] = P_((i / 3 + idxstartcorr_q) * nErrorStatesAtCompileTime + i % 3);

	for (int i = 0; i < 9; i++)
		cov[(i / 3 + 3) * 6 + (i % 3 + 3)] = P_((i / 3 + idxstartcorr_q) * nErrorStatesAtCompileTime + (i % 3 + 6));
}


};
