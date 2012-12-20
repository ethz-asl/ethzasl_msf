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

#include <msf_core/msf_core.hpp>

namespace msf_core{

template<class H_type, class Res_type, class R_type>
void MSF_MeasurementBase::calculateAndApplyCorrection(boost::shared_ptr<EKFState_T> state, MSF_Core<EKFState_T>& core, const Eigen::MatrixBase<H_type>& H_delayed,
                                                      const Eigen::MatrixBase<Res_type> & res_delayed, const Eigen::MatrixBase<R_type>& R_delayed)
{


  EIGEN_STATIC_ASSERT_FIXED_SIZE(H_type);
  EIGEN_STATIC_ASSERT_FIXED_SIZE(R_type);

  // get measurements
  /// correction from EKF update
  Eigen::Matrix<double, MSF_Core<EKFState_T>::nErrorStatesAtCompileTime, 1> correction_;

  R_type S;
  Eigen::Matrix<double, MSF_Core<EKFState_T>::nErrorStatesAtCompileTime, R_type::RowsAtCompileTime> K;
  MSF_Core<EKFState_T>::ErrorStateCov & P = state->P_;

  S = H_delayed * P * H_delayed.transpose() + R_delayed;
  K = P * H_delayed.transpose() * S.inverse();

  correction_ = K * res_delayed;
  const MSF_Core<EKFState_T>::ErrorStateCov KH = (MSF_Core<EKFState_T>::ErrorStateCov::Identity() - K * H_delayed);
  P = KH * P * KH.transpose() + K * R_delayed * K.transpose();

  // make sure P stays symmetric
  P = 0.5 * (P + P.transpose());

  core.applyCorrection(state, correction_);
}

void MSF_InitMeasurement::apply(boost::shared_ptr<EKFState_T> stateWithCovariance, MSF_Core<EKFState_T>& core){


  stateWithCovariance->time_ = ros::Time::now().toSec(); //makes this state a valid starting point

  boost::fusion::for_each(
      stateWithCovariance->statevars_,
      msf_tmp::copyInitStates<EKFState_T>(InitState)
  );

  if(! (InitState.P_.minCoeff() == 0 && InitState.P_.maxCoeff() == 0)){
    stateWithCovariance->P_ = InitState.P_;
    ROS_WARN_STREAM("Using user defined initial error state covariance");
  }else{
    ROS_WARN_STREAM("Using simulated core plus fixed diag initial error state covariance");
  }

  if(ContainsInitialSensorReadings_){
    stateWithCovariance->a_m_ = InitState.a_m_;
    stateWithCovariance->w_m_ = InitState.w_m_;
  }else{
    stateWithCovariance->a_m_.setZero();
    stateWithCovariance->w_m_.setZero();
  }
  core.initExternalPropagation(stateWithCovariance);

}

}
