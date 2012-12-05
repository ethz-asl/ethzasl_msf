/*

Copyright (c) 2012, Simon Lynen, ASL, ETH Zurich, Switzerland
You can contact the author at <slynen at ethz dot org>

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

#include <msf_core/msf_core.hpp>

namespace msf_core{

/// main update routine called by a given sensor, will apply the measurement to the core
template<class H_type, class Res_type, class R_type>
void MSF_MeasurementBase::calculateAndApplyCorrection(boost::shared_ptr<EKFState> state, MSF_Core& core, const Eigen::MatrixBase<H_type>& H_delayed,
		const Eigen::MatrixBase<Res_type> & res_delayed, const Eigen::MatrixBase<R_type>& R_delayed)
{

	if(state->time_ <= 0){ //is the state valid?
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
    ROS_INFO_STREAM("covariance for meas "<<(P.block<3,3>(0,0)));

    ROS_INFO_STREAM("H_delayed "<<(H_delayed.template block<3,3>(0,0)));
    ROS_INFO_STREAM("R_delayed "<<(R_delayed.template block<3,3>(0,0)));

	S = H_delayed * P * H_delayed.transpose() + R_delayed;
	K = P * H_delayed.transpose() * S.inverse();

	correction_ = K * res_delayed;
	const MSF_Core::ErrorStateCov KH = (MSF_Core::ErrorStateCov::Identity() - K * H_delayed);
	P = KH * P * KH.transpose() + K * R_delayed * K.transpose();

	// make sure P stays symmetric
	P = 0.5 * (P + P.transpose());
    ROS_INFO_STREAM("covariance after meas "<<(P.block<3,3>(0,0)));

	core.applyCorrection(state, correction_);
}

void MSF_InitMeasurement::apply(boost::shared_ptr<EKFState> stateWithCovariance, MSF_Core& core){

	ROS_INFO_STREAM("Applying init meas");

	stateWithCovariance->time_ = ros::Time::now().toSec(); //makes this state a valid starting point

	boost::fusion::for_each(
			stateWithCovariance->statevars_,
			msf_tmp::copyInitStates<EKFState>(InitState)
	);

	stateWithCovariance->P_ = InitState.P_;

	if(ContainsInitialSensorReadings_){
		stateWithCovariance->a_m_ = InitState.a_m_;
		stateWithCovariance->w_m_ = InitState.w_m_;
	}
	core.initExternalPropagation(stateWithCovariance);

}

}
