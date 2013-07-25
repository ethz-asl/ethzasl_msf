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

#include <msf_core/msf_core.h>

namespace msf_core {
template<typename EKFState_T>
MSF_MeasurementBase<EKFState_T>::MSF_MeasurementBase(bool isabsoluteMeasurement,
                                                     int sensorID)
    : sensorID_(sensorID),
      isabsolute_(isabsoluteMeasurement),
      time(0) { }

template<typename EKFState_T>
template<class H_type, class Res_type, class R_type>
void MSF_MeasurementBase<EKFState_T>::calculateAndApplyCorrection(
    shared_ptr<EKFState_T> state, MSF_Core<EKFState_T>& core,
    const Eigen::MatrixBase<H_type>& H_delayed,
    const Eigen::MatrixBase<Res_type> & res_delayed,
    const Eigen::MatrixBase<R_type>& R_delayed) {

  EIGEN_STATIC_ASSERT_FIXED_SIZE(H_type);
  EIGEN_STATIC_ASSERT_FIXED_SIZE(R_type);

  // Get measurements.
  /// Correction from EKF update.
  Eigen::Matrix<double, MSF_Core<EKFState_T>::nErrorStatesAtCompileTime, 1> correction_;

  R_type S;
  Eigen::Matrix<double, MSF_Core<EKFState_T>::nErrorStatesAtCompileTime,
      R_type::RowsAtCompileTime> K;
  typename MSF_Core<EKFState_T>::ErrorStateCov & P = state->P;

  S = H_delayed * P * H_delayed.transpose() + R_delayed;
  K = P * H_delayed.transpose() * S.inverse();

  correction_ = K * res_delayed;
  const typename MSF_Core<EKFState_T>::ErrorStateCov KH =
      (MSF_Core<EKFState_T>::ErrorStateCov::Identity() - K * H_delayed);
  P = KH * P * KH.transpose() + K * R_delayed * K.transpose();

  // Make sure P stays symmetric.
  P = 0.5 * (P + P.transpose());

  core.applyCorrection(state, correction_);
}

template<typename EKFState_T>
void MSF_MeasurementBase<EKFState_T>::calculateAndApplyCorrection(
    shared_ptr<EKFState_T> state, MSF_Core<EKFState_T>& core,
    const Eigen::MatrixXd& H_delayed, const Eigen::MatrixXd & res_delayed,
    const Eigen::MatrixXd& R_delayed) {

  // Get measurements.
  /// Correction from EKF update.
  Eigen::Matrix<double, MSF_Core<EKFState_T>::nErrorStatesAtCompileTime, 1> correction_;

  Eigen::MatrixXd S;
  Eigen::MatrixXd K(
      static_cast<int>(MSF_Core<EKFState_T>::nErrorStatesAtCompileTime),
      R_delayed.rows());
  typename MSF_Core<EKFState_T>::ErrorStateCov & P = state->P;

  S = H_delayed * P * H_delayed.transpose() + R_delayed;
  K = P * H_delayed.transpose() * S.inverse();

  correction_ = K * res_delayed;
  const typename MSF_Core<EKFState_T>::ErrorStateCov KH =
      (MSF_Core<EKFState_T>::ErrorStateCov::Identity() - K * H_delayed);
  P = KH * P * KH.transpose() + K * R_delayed * K.transpose();

  // Make sure P stays symmetric.
  P = 0.5 * (P + P.transpose());

  core.applyCorrection(state, correction_);
}

template<typename EKFState_T>
template<class H_type, class Res_type, class R_type>
void MSF_MeasurementBase<EKFState_T>::calculateAndApplyCorrectionRelative(
    shared_ptr<EKFState_T> state_old,
    shared_ptr<EKFState_T> state_new, MSF_Core<EKFState_T>& core,
    const Eigen::MatrixBase<H_type>& H_old,
    const Eigen::MatrixBase<H_type>& H_new,
    const Eigen::MatrixBase<Res_type> & res,
    const Eigen::MatrixBase<R_type>& R) {

  EIGEN_STATIC_ASSERT_FIXED_SIZE(H_type);
  EIGEN_STATIC_ASSERT_FIXED_SIZE(R_type);

  // Get measurements.
  /// Correction from EKF update.
  Eigen::Matrix<double, MSF_Core<EKFState_T>::nErrorStatesAtCompileTime, 1> correction_;

  enum {
    Pdim = MSF_Core<EKFState_T>::nErrorStatesAtCompileTime
  };

  // Get the accumulated system dynamics.
  Eigen::Matrix<double, Pdim, Pdim> F_accum;
  core.getAccumF_SC(state_old, state_new, F_accum);

  /*[
   *        | P_kk       P_kk * F' |
   * P_SC = |                      |
   *        | F * P_kk   P_mk      |
   */
  Eigen::Matrix<double, 2 * Pdim, 2 * Pdim> P_SC;
  P_SC.template block<Pdim, Pdim>(0, 0) = state_old->P;
  // According to TRO paper, ICRA paper has a mistake here.
  P_SC.template block<Pdim, Pdim>(0, Pdim) = state_old->P * F_accum.transpose();
  P_SC.template block<Pdim, Pdim>(Pdim, 0) = F_accum * state_old->P;
  P_SC.template block<Pdim, Pdim>(Pdim, Pdim) = state_new->P;

  /*
   * H_SC = [H_kk  H_mk]
   */
  Eigen::Matrix<double, H_type::RowsAtCompileTime, H_type::ColsAtCompileTime * 2> H_SC;

  H_SC.template block<H_type::RowsAtCompileTime,
      H_type::ColsAtCompileTime>(0, 0) = H_old;
  H_SC.template block<H_type::RowsAtCompileTime, H_type::ColsAtCompileTime>(
      0, H_type::ColsAtCompileTime) = H_new;

  R_type S_SC;
  S_SC = H_SC * P_SC * H_SC.transpose() + R;

  //TODO (slynen): Only compute the part of K_SC that we need.
  Eigen::Matrix<double, MSF_Core<EKFState_T>::nErrorStatesAtCompileTime * 2,
      R_type::RowsAtCompileTime> K_SC;
  K_SC = P_SC * H_SC.transpose() * S_SC.inverse();

  Eigen::Matrix<double, MSF_Core<EKFState_T>::nErrorStatesAtCompileTime,
      R_type::RowsAtCompileTime> K;
  K = K_SC
      .template block<MSF_Core<EKFState_T>::nErrorStatesAtCompileTime,
          R_type::RowsAtCompileTime>(
      MSF_Core<EKFState_T>::nErrorStatesAtCompileTime, 0);

  correction_ = K * res;

  typename MSF_Core<EKFState_T>::ErrorStateCov & P = state_new->P;
  P = P - K * S_SC * K.transpose();

  // Make sure P stays symmetric.
  // TODO (slynen): EV, set Evalues<eps to zero, then reconstruct.
  state_new->P = 0.5 * (state_new->P + state_new->P.transpose());

  core.applyCorrection(state_new, correction_);
}

template<typename EKFState_T>
void MSF_InitMeasurement<EKFState_T>::apply(
    shared_ptr<EKFState_T> stateWithCovariance,
    MSF_Core<EKFState_T>& core) {

  // Makes this state a valid starting point.
  stateWithCovariance->time = this->time;

  boost::fusion::for_each(stateWithCovariance->statevars,
                          msf_tmp::copyInitStates<EKFState_T>(InitState));

  if (!(InitState.P.minCoeff() == 0 && InitState.P.maxCoeff() == 0)) {
    stateWithCovariance->P = InitState.P;
    MSF_WARN_STREAM("Using user defined initial error state covariance.");
  } else {
    MSF_WARN_STREAM(
        "Using simulated core plus fixed diag initial error state covariance.");
  }

  if (ContainsInitialSensorReadings_) {
    stateWithCovariance->a_m = InitState.a_m;
    stateWithCovariance->w_m = InitState.w_m;
  } else {
    stateWithCovariance->a_m.setZero();
    stateWithCovariance->w_m.setZero();
  }

  core.usercalc().publishStateInitial(stateWithCovariance);

}
}
