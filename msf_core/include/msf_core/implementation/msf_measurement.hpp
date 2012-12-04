/*
 * msf_measurement.hpp
 *
 *  Created on: Nov 13, 2012
 *      Author: slynen
 */

#include <msf_core/msf_core.hpp>

namespace msf_core{

/// main update routine called by a given sensor, will apply the measurement to the core
template<class H_type, class Res_type, class R_type>
void MSF_MeasurementBase::calculateAndApplyCorrection(boost::shared_ptr<EKFState> state, MSF_Core& core, const Eigen::MatrixBase<H_type>& H_delayed,
		const Eigen::MatrixBase<Res_type> & res_delayed, const Eigen::MatrixBase<R_type>& R_delayed)
{

	if(state->time_ <= 0){ //is the state valid? (init changes the time)
		return;
	}

	EIGEN_STATIC_ASSERT_FIXED_SIZE(H_type);
	EIGEN_STATIC_ASSERT_FIXED_SIZE(R_type);

	// get measurements
	/// correction from EKF update
	Eigen::Matrix<double, MSF_Core::nErrorStatesAtCompileTime, 1> correction_;

	R_type S;
	Eigen::Matrix<double, MSF_Core::nErrorStatesAtCompileTime, R_type::RowsAtCompileTime> K;
	MSF_Core::ErrorStateCov & P = state->P_;

	S = H_delayed * P * H_delayed.transpose() + R_delayed;
	K = P * H_delayed.transpose() * S.inverse();

	correction_ = K * res_delayed;
	const MSF_Core::ErrorStateCov KH = (MSF_Core::ErrorStateCov::Identity() - K * H_delayed);
	P = KH * P * KH.transpose() + K * R_delayed * K.transpose();

	// make sure P stays symmetric
	P = 0.5 * (P + P.transpose());

	core.applyCorrection(state, correction_);
}

void MSF_InitMeasurement::apply(boost::shared_ptr<EKFState> stateWithCovariance, MSF_Core& core){

	stateWithCovariance->time_ = ros::Time::now().toSec(); //makes this state a valid starting point

	boost::fusion::for_each(
			stateWithCovariance->statevars_,
			msf_tmp::copyInitStates<EKFState>(InitState)
	);

	if(ContainsInitialSensorReadings_){
		stateWithCovariance->P_ = InitState.P_;
		stateWithCovariance->a_m_ = InitState.a_m_;
		stateWithCovariance->w_m_ = InitState.w_m_;
	}
	core.initExternalPropagation(stateWithCovariance);

}

}
