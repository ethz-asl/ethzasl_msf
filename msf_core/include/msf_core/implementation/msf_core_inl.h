/*
 * Copyright (C) 2012-2013 Simon Lynen, ASL, ETH Zurich, Switzerland
 * You can contact the author at <slynen at ethz dot ch>
 * Copyright (C) 2011-2012 Stephan Weiss, ASL, ETH Zurich, Switzerland
 * You can contact the author at <stephan dot weiss at ieee dot org>
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
#ifndef MSF_CORE_INL_H_
#define MSF_CORE_INL_H_

#include <chrono>
#include <thread>
#include <deque>
#include <numeric>
#include <string>
#include <vector>

#include <msf_timing/Timer.h>
#include <msf_core/implementation/calcQCore.h>
#include <msf_core/eigen_utils.h>
#include <msf_core/msf_sensormanager.h>
#include <msf_core/msf_tools.h>
#include <msf_core/msf_measurement.h>
#include <msf_core/falsecolor.h>
#include <msf_core/msf_types.h>

namespace msf_core {
template<typename EKFState_T>
MSF_Core<EKFState_T>::MSF_Core(const MSF_SensorManager<EKFState_T>& GetUserCalc)
    : usercalc_(GetUserCalc) {  // The interface for the user to customize EKF
  // interna, DO ABSOLUTELY NOT USE THIS POINTER INSIDE THIS CTOR!!

  // Set the output precision for numeric values.
  std::setprecision(NUMERIC_PREC);
  initialized_ = false;
  predictionMade_ = false;
  isfuzzyState_ = false;
  time_P_propagated = 0;
  it_last_IMU = stateBuffer_.GetIteratorEnd();
}

template<typename EKFState_T>
MSF_Core<EKFState_T>::~MSF_Core() {
}

template<typename EKFState_T>
void MSF_Core<EKFState_T>::SetPCore(
    Eigen::Matrix<double, EKFState_T::nErrorStatesAtCompileTime,
        EKFState_T::nErrorStatesAtCompileTime>& P) {
  enum {
    // We might want to calculate this, but on the other hand the values for the
    // matrix later on are anyway hardcoded.
    coreErrorStates = 15
  };
  P.setIdentity();
  P *= 0.0001;  // Set diagonal small covariance for all states.

  // Now set the core state covariance to the simulated values.
  Eigen::Matrix<double, coreErrorStates, coreErrorStates> P_core;
  // This violates the 80 chars rule, to keep it somehow readable.
  P_core << 0.0166, 0.0122, -0.0015, 0.0211, 0.0074, 0.0000, 0.0012, -0.0012, 0.0001, -0.0000, 0.0000, -0.0000, -0.0003, -0.0002, -0.0000,
            0.0129, 0.0508, -0.0020, 0.0179, 0.0432, 0.0006, 0.0020, 0.0004, -0.0002, -0.0000, 0.0000, 0.0000, 0.0003, -0.0002, 0.0000,
            -0.0013, -0.0009, 0.0142, -0.0027, 0.0057, 0.0079, 0.0007, 0.0007, 0.0000, -0.0000, -0.0000, 0.0000, -0.0001, -0.0004, -0.0001,
            0.0210, 0.0162, -0.0026, 0.0437, 0.0083, -0.0017, 0.0016, -0.0021, -0.0014, -0.0000, 0.0000, 0.0000, 0.0003, -0.0001, 0.0000,
            0.0093, 0.0461, 0.0036, 0.0153, 0.0650, -0.0016, 0.0025, 0.0013, -0.0000, -0.0000, 0.0000, 0.0000, 0.0003, 0.0002, 0.0000,
            -0.0000, 0.0005, 0.0080, -0.0019, -0.0021, 0.0130, 0.0001, 0.0001, 0.0000, -0.0000, 0.0000, -0.0000, -0.0003, 0.0001, -0.0001,
            0.0012, 0.0024, 0.0006, 0.0017, 0.0037, 0.0001, 0.0005, 0.0000, 0.0001, -0.0000, 0.0000, -0.0000, -0.0000, -0.0001, -0.0000,
            -0.0011, 0.0008, 0.0007, -0.0023, 0.0019, 0.0001, 0.0000, 0.0005, -0.0001, -0.0000, -0.0000, 0.0000, 0.0001, -0.0001, -0.0000,
            0.0001, -0.0002, -0.0000, -0.0014, 0.0001, 0.0000, 0.0000, -0.0001, 0.0006, -0.0000, -0.0000, -0.0000, 0.0000, 0.0000, -0.0000,
            -0.0000, -0.0000, -0.0000, -0.0000, -0.0000, -0.0000, -0.0000, -0.0000, -0.0000, 0.0000, -0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
            0.0000, 0.0000, -0.0000, 0.0000, 0.0000, 0.0000, 0.0000, -0.0000, -0.0000, -0.0000, 0.0000, -0.0000, 0.0000, 0.0000, 0.0000,
            -0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, -0.0000, 0.0000, -0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, -0.0000,
            -0.0003, 0.0003, -0.0001, 0.0003, 0.0003, -0.0003, -0.0000, 0.0001, 0.0000, 0.0000, 0.0000, 0.0000, 0.0010, 0.0000, 0.0000,
            -0.0002, -0.0002, -0.0004, -0.0001, 0.0003, 0.0001, -0.0001, -0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0010, 0.0000,
            -0.0000, 0.0000, -0.0001, 0.0000, 0.0000, -0.0001, -0.0000, -0.0000, -0.0000, 0.0000, 0.0000, -0.0000, 0.0000, 0.0000, 0.0001;

  P_core = 0.5 * (P_core + P_core.transpose());
  P.template block<coreErrorStates, coreErrorStates>(0, 0) = P_core;
}

template<typename EKFState_T>
const MSF_SensorManager<EKFState_T>& MSF_Core<EKFState_T>::GetUserCalc() const {
  return usercalc_;
}

template<typename EKFState_T>
void MSF_Core<EKFState_T>::ProcessIMU(
    const msf_core::Vector3& linear_acceleration,
    const msf_core::Vector3& angular_velocity, const double& msg_stamp,
    size_t /*msg_seq*/) {

  if (!initialized_)
    return;

  msf_timing::DebugTimer timer_PropGetClosestState("PropGetClosestState");
  if (it_last_IMU == stateBuffer_.GetIteratorEnd()) {
    it_last_IMU = stateBuffer_.GetIteratorClosestBefore(msg_stamp);
  }

  shared_ptr<EKFState_T> lastState = it_last_IMU->second;
  timer_PropGetClosestState.Stop();

  msf_timing::DebugTimer timer_PropPrepare("PropPrepare");
  if (lastState->time == constants::INVALID_TIME) {
    MSF_WARN_STREAM_THROTTLE(
        2, __FUNCTION__<<"ImuCallback: closest state is invalid\n");
    return;  // Early abort.
  }

  shared_ptr<EKFState_T> currentState(new EKFState_T);
  currentState->time = msg_stamp;

  // Check if this IMU message is really after the last one (caused by restarting
  // a bag file).
  if (currentState->time - lastState->time < -0.01 && predictionMade_) {
    initialized_ = false;
    predictionMade_ = false;
    MSF_ERROR_STREAM(
        __FUNCTION__<<"latest IMU message was out of order by a too large amount, "
        "resetting EKF: last-state-time: " << msf_core::timehuman(lastState->time)
        << " "<< "current-imu-time: "<< msf_core::timehuman(currentState->time));
    return;
  }

  static int seq = 0;
  // Get inputs.
  currentState->a_m = linear_acceleration;
  currentState->w_m = angular_velocity;
  currentState->noise_gyr = Vector3::Constant(usercalc_.GetParamNoiseGyr());
  currentState->noise_acc = Vector3::Constant(usercalc_.GetParamNoiseAcc());


  // Remove acc spikes (TODO (slynen): find a cleaner way to do this).
  static Eigen::Matrix<double, 3, 1> last_am =
      Eigen::Matrix<double, 3, 1>(0, 0, 0);
  if (currentState->a_m.norm() > 50)
    currentState->a_m = last_am;
  else {
    // Try to get the state before the current time.
    if (lastState->time == constants::INVALID_TIME) {
      MSF_WARN_STREAM(
          "Accelerometer readings had a spike, but no prior state was in the "
          "buffer to take cleaner measurements from");
      return;
    }
    last_am = lastState->a_m;
  }
  if (!predictionMade_) {
    if (lastState->time == constants::INVALID_TIME) {
      MSF_WARN_STREAM("Wanted to compare prediction time offset to last state, "
      "but no prior state was in the buffer to take cleaner measurements from");
      return;
    }
    if (fabs(currentState->time - lastState->time) > 0.1) {
      MSF_WARN_STREAM_THROTTLE(
          2, "large time-gap re-initializing to last state\n");
      typename StateBuffer_T::Ptr_T tmp = stateBuffer_.UpdateTime(
          lastState->time, currentState->time);
      time_P_propagated = currentState->time;
      return;  // // early abort // // (if timegap too big)
    }
  }

  if (lastState->time == constants::INVALID_TIME) {
    MSF_WARN_STREAM(
        "Wanted to propagate state, but no valid prior state could be found in "
        "the buffer");
    return;
  }
  timer_PropPrepare.Stop();

  msf_timing::DebugTimer timer_PropState("PropState");
  //propagate state and covariance
  PropagateState(lastState, currentState);
  timer_PropState.Stop();

  msf_timing::DebugTimer timer_PropInsertState("PropInsertState");
  it_last_IMU = stateBuffer_.Insert(currentState);
  timer_PropInsertState.Stop();

  msf_timing::DebugTimer timer_PropCov("PropCov");
  PropagatePOneStep();
  timer_PropCov.Stop();
  usercalc_.PublishStateAfterPropagation(currentState);

  // Making sure we have sufficient states to apply measurements to.
  if (stateBuffer_.Size() > 3)
    predictionMade_ = true;

  if (predictionMade_) {
    // Check if we can apply some pending measurement.
    HandlePendingMeasurements();
  }
  seq++;

}

template<typename EKFState_T>
void MSF_Core<EKFState_T>::ProcessExternallyPropagatedState(
    const msf_core::Vector3& linear_acceleration,
    const msf_core::Vector3& angular_velocity, const msf_core::Vector3& p,
    const msf_core::Vector3& v, const msf_core::Quaternion& q,
    bool is_already_propagated, const double& msg_stamp, size_t /*msg_seq*/) {

  if (!initialized_)
    return;

  // fast method to get last_IMU is broken
  // TODO(slynen): fix iterator setting for state callback

//  // Get the closest state and check validity.
//  if (it_last_IMU == stateBuffer_.getIteratorEnd()) {
//    it_last_IMU = stateBuffer_.getIteratorClosestBefore(msg_stamp);
//    assert(!(it_last_IMU == stateBuffer_.getIteratorEnd()));
//  }

  // TODO(slynen): not broken, revert back when it_last_IMU->second from above is fixed
  shared_ptr<EKFState_T> lastState = stateBuffer_.GetLast();//it_last_IMU->second;
  if (lastState->time == constants::INVALID_TIME) {
    MSF_WARN_STREAM_THROTTLE(2, "StateCallback: closest state is invalid\n");
    return;  // Early abort.
  }

  // Create a new state.
  shared_ptr<EKFState_T> currentState(new EKFState_T);
  currentState->time = msg_stamp;

  // Get inputs.
  currentState->a_m = linear_acceleration;
  currentState->w_m = angular_velocity;
  currentState->noise_gyr = Vector3::Constant(usercalc_.GetParamNoiseGyr());
  currentState->noise_acc = Vector3::Constant(usercalc_.GetParamNoiseAcc());

  // Remove acc spikes (TODO (slynen): Find a cleaner way to do this).
  static Eigen::Matrix<double, 3, 1> last_am =
      Eigen::Matrix<double, 3, 1>(0, 0, 0);
  if (currentState->a_m.norm() > 50)
    currentState->a_m = last_am;
  else
    last_am = currentState->a_m;

  if (!predictionMade_) {
    if (fabs(currentState->time - lastState->time) > 5) {
      typename StateBuffer_T::Ptr_T tmp = stateBuffer_.UpdateTime(
          lastState->time, currentState->time);
      MSF_WARN_STREAM_THROTTLE(
          2, "large time-gap re-initializing to last state: "
          << msf_core::timehuman(tmp->time));
      return;  // Early abort (if timegap too big).
    }
  }

  bool isnumeric = CheckForNumeric(
      currentState->template Get<StateDefinition_T::p>(),
      "before prediction p");

  // State propagation is made externally, so we read the actual state.
  if (is_already_propagated && isnumeric) {
    currentState->template Get<StateDefinition_T::p>() = p;
    currentState->template Get<StateDefinition_T::v>() = v;
    currentState->template Get<StateDefinition_T::q>() = q;

    // Zero props: copy non propagation states from last state.
    boost::fusion::for_each(
        currentState->statevars,
        msf_tmp::CopyNonPropagationStates<EKFState_T>(*lastState));

  } else if (!is_already_propagated || !isnumeric) {
    // Otherwise let's do the state prop. here.
    PropagateState(lastState, currentState);
  }

  // Clean reset of state and measurement buffer, before we start propagation.
  if (!predictionMade_) {

    // Make sure we keep the covariance for the first state.
    currentState->P = stateBuffer_.GetLast()->P;
    time_P_propagated = currentState->time;

    stateBuffer_.Clear();
    MeasurementBuffer_.Clear();
    while (!queueFutureMeasurements_.empty()) {
      queueFutureMeasurements_.pop();
    }
  }

  stateBuffer_.Insert(currentState);
  PropagatePOneStep();
  predictionMade_ = true;
  usercalc_.PublishStateAfterPropagation(currentState);

  isnumeric = CheckForNumeric(
      currentState->template Get<StateDefinition_T::p>(), "prediction p");
  isnumeric = CheckForNumeric(currentState->P, "prediction done P");

  // Check if we can apply some pending measurement.
  HandlePendingMeasurements();
}

template<typename EKFState_T>
void MSF_Core<EKFState_T>::HandlePendingMeasurements() {
  if (queueFutureMeasurements_.empty())
    return;
  shared_ptr<MSF_MeasurementBase<EKFState_T> > meas = queueFutureMeasurements_
      .front();
  queueFutureMeasurements_.pop();
  AddMeasurement(meas);
}

template<typename EKFState_T>
void MSF_Core<EKFState_T>::CleanUpBuffers() {
  double timeold = 60;  // 1 min.
  stateBuffer_.ClearOlderThan(timeold);
  MeasurementBuffer_.ClearOlderThan(timeold);
}

template<typename EKFState_T>
void MSF_Core<EKFState_T>::PropagateState(shared_ptr<EKFState_T>& state_old,
                                          shared_ptr<EKFState_T>& state_new) {

  double dt = state_new->time - state_old->time;

  // Reset new state to zero.
  boost::fusion::for_each(state_new->statevars, msf_tmp::ResetState());

  // Zero props: copy constant for non propagated states.
  boost::fusion::for_each(
      state_new->statevars,
      msf_tmp::CopyNonPropagationStates<EKFState_T>(*state_old));

  Eigen::Matrix<double, 3, 1> dv;
  const Vector3 ew = state_new->w_m
      - state_new->template Get<StateDefinition_T::b_w>();
  const Vector3 ewold = state_old->w_m
      - state_old->template Get<StateDefinition_T::b_w>();
  const Vector3 ea = state_new->a_m
      - state_new->template Get<StateDefinition_T::b_a>();
  const Vector3 eaold = state_old->a_m
      - state_old->template Get<StateDefinition_T::b_a>();
  const Matrix4 Omega = OmegaMatJPL(ew);
  const Matrix4 OmegaOld = OmegaMatJPL(ewold);
  Matrix4 OmegaMean = OmegaMatJPL((ew + ewold) / 2);

  // Zero order quaternion integration.
  // cur_state.q_ = (Eigen::Matrix<double,4,4>::Identity() +
  // 0.5*Omega*dt)*StateBuffer_[(unsigned char)(idx_state_-1)].q_.coeffs();

  // First order quaternion integration, this is kind of costly and may not add
  // a lot to the quality of propagation...
  int div = 1;
  Matrix4 MatExp;
  MatExp.setIdentity();
  OmegaMean *= 0.5 * dt;
  for (int i = 1; i < 5; i++) {  // Can be made fourth order or less to save cycles.
    div *= i;
    MatExp = MatExp + OmegaMean / div;
    OmegaMean *= OmegaMean;
  }

  // First oder quat integration matrix.
  const Matrix4 quat_int =
      MatExp + 1.0 / 48.0 * (Omega * OmegaOld - OmegaOld * Omega) * dt * dt;

  // First oder quaternion integration.
  state_new->template Get<StateDefinition_T::q>().coeffs() = quat_int *
  state_old->template Get<StateDefinition_T::q>().coeffs();
  state_new->template Get<StateDefinition_T::q>().normalize();

  dv = (state_new->template Get<StateDefinition_T::q>().toRotationMatrix() *
       ea + state_old->template Get<StateDefinition_T::q>().toRotationMatrix() *
       eaold) / 2;
  state_new->template Get<StateDefinition_T::v>() =
      state_old->template Get<StateDefinition_T::v>()
      + (dv - constants::GRAVITY) * dt;
  state_new->template Get<StateDefinition_T::p>() =
      state_old->template Get<StateDefinition_T::p>()
      + ((state_new->template Get<StateDefinition_T::v>()
          + state_old->template Get<StateDefinition_T::v>()) / 2 * dt);
}

template<typename EKFState_T>
void MSF_Core<EKFState_T>::PropagatePOneStep() {
  // Also propagate the covariance one step further, to distribute the
  // processing load over time.
  typename StateBuffer_T::iterator_T stateIteratorPLastPropagated = stateBuffer_
      .GetIteratorAtValue(time_P_propagated, false);
  typename StateBuffer_T::iterator_T stateIteratorPLastPropagatedNext =
      stateIteratorPLastPropagated;

  ++stateIteratorPLastPropagatedNext;
  // Might happen if there is a measurement in the future.
  if (stateIteratorPLastPropagatedNext != stateBuffer_.GetIteratorEnd()) {

    PredictProcessCovariance(stateIteratorPLastPropagated->second,
                             stateIteratorPLastPropagatedNext->second);

    if (!CheckForNumeric(
        stateIteratorPLastPropagatedNext->second
            ->template Get<StateDefinition_T::p>(),
        "prediction p")) {
      MSF_WARN_STREAM(
          "prop state from:\t"<<
          stateIteratorPLastPropagated->second->ToEigenVector());
      MSF_WARN_STREAM(
          "prop state to:\t"<<
          stateIteratorPLastPropagatedNext->second->ToEigenVector());
      MSF_ERROR_STREAM(__FUNCTION__<<" Resetting EKF");
      predictionMade_ = initialized_ = false;
    }
  }
}

template<typename EKFState_T>
void MSF_Core<EKFState_T>::GetAccumulatedStateTransitionStochasticCloning(
    const shared_ptr<EKFState_T>& state_old,
    const shared_ptr<EKFState_T>& state_new,
    Eigen::Matrix<double, MSF_Core<EKFState_T>::nErrorStatesAtCompileTime,
        MSF_Core<EKFState_T>::nErrorStatesAtCompileTime>& F) {
  typename StateBuffer_T::iterator_T it = stateBuffer_.GetIteratorAtValue(
      state_old);
  typename StateBuffer_T::iterator_T itend = stateBuffer_.GetIteratorAtValue(
      state_new);
  typedef Eigen::Matrix<double, MSF_Core<EKFState_T>::nErrorStatesAtCompileTime,
      MSF_Core<EKFState_T>::nErrorStatesAtCompileTime> F_type;
  F = F_type::Identity();
  for (; it != itend; ++it) {
    F = F * it->second->Fd;
  }
}

template<typename EKFState_T>
void MSF_Core<EKFState_T>::PredictProcessCovariance(
    shared_ptr<EKFState_T>& state_old, shared_ptr<EKFState_T>& state_new) {

  double dt = state_new->time - state_old->time;

  if (dt <= 0) {
    MSF_WARN_STREAM_THROTTLE(
        1, "Requested cov prop between two states that where "<<dt<<" "
        "seconds apart. Rejecting");
    return;
  }

  // Noises.
  const Vector3 nav = Vector3::Constant(usercalc_.GetParamNoiseAcc());
  const Vector3 nbav = Vector3::Constant(usercalc_.GetParamNoiseAccbias());

  const Vector3 nwv = Vector3::Constant(usercalc_.GetParamNoiseGyr());
  const Vector3 nbwv = Vector3::Constant(usercalc_.GetParamNoiseGyrbias());

  // Bias corrected IMU readings.
  const Vector3 ew = state_new->w_m
      - state_new->template Get<StateDefinition_T::b_w>();
  const Vector3 ea = state_new->a_m
      - state_new->template Get<StateDefinition_T::b_a>();

  const Matrix3 a_sk = Skew(ea);
  const Matrix3 w_sk = Skew(ew);
  const Matrix3 eye3 = Eigen::Matrix<double, 3, 3>::Identity();

  const Matrix3 C_eq = state_new->template Get<StateDefinition_T::q>().toRotationMatrix();

  const double dt_p2_2 = dt * dt * 0.5;
  const double dt_p3_6 = dt_p2_2 * dt / 3.0;
  const double dt_p4_24 = dt_p3_6 * dt * 0.25;
  const double dt_p5_120 = dt_p4_24 * dt * 0.2;

  const Matrix3 Ca3 = C_eq * a_sk;
  const Matrix3 A = Ca3
      * (-dt_p2_2 * eye3 + dt_p3_6 * w_sk - dt_p4_24 * w_sk * w_sk);
  const Matrix3 B = Ca3
      * (dt_p3_6 * eye3 - dt_p4_24 * w_sk + dt_p5_120 * w_sk * w_sk);
  const Matrix3 D = -A;
  const Matrix3 E = eye3 - dt * w_sk + dt_p2_2 * w_sk * w_sk;
  const Matrix3 F = -dt * eye3 + dt_p2_2 * w_sk - dt_p3_6 * (w_sk * w_sk);
  const Matrix3 C = Ca3 * F;

  // Discrete error state propagation Matrix Fd according to:
  // Stephan Weiss and Roland Siegwart.
  // Real-Time Metric State Estimation for Modular Vision-Inertial Systems.
  // IEEE International Conference on Robotics and Automation. Shanghai, China, 2011
  typename EKFState_T::F_type& Fd = state_old->Fd;

  enum {
    idxstartcorr_p = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
        StateDefinition_T::p>::value,
    idxstartcorr_v = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
        StateDefinition_T::v>::value,
    idxstartcorr_q = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
        StateDefinition_T::q>::value,
    idxstartcorr_b_w = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
        StateDefinition_T::b_w>::value,
    idxstartcorr_b_a = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
        StateDefinition_T::b_a>::value
  };

  Fd.template block<3, 3>(idxstartcorr_p, idxstartcorr_v) = dt * eye3;
  Fd.template block<3, 3>(idxstartcorr_p, idxstartcorr_q) = A;
  Fd.template block<3, 3>(idxstartcorr_p, idxstartcorr_b_w) = B;
  Fd.template block<3, 3>(idxstartcorr_p, idxstartcorr_b_a) = -C_eq * dt_p2_2;

  Fd.template block<3, 3>(idxstartcorr_v, idxstartcorr_q) = C;
  Fd.template block<3, 3>(idxstartcorr_v, idxstartcorr_b_w) = D;
  Fd.template block<3, 3>(idxstartcorr_v, idxstartcorr_b_a) = -C_eq * dt;

  Fd.template block<3, 3>(idxstartcorr_q, idxstartcorr_q) = E;
  Fd.template block<3, 3>(idxstartcorr_q, idxstartcorr_b_w) = F;

  typename EKFState_T::Q_type& Qd = state_old->Qd;

  CalcQCore<StateSequence_T, StateDefinition_T>(
      dt, state_new->template Get<StateDefinition_T::q>(), ew, ea, nav, nbav,
      nwv, nbwv, Qd);

  // Call user Q calc to fill in the blocks of auxiliary states.
  // TODO optim: make state Q-blocks map respective parts of Q using Eigen Map,
  // avoids copy.
  usercalc_.CalculateQAuxiliaryStates(*state_new, dt);

  // Now copy the userdefined blocks to Qd.
  boost::fusion::for_each(
      state_new->statevars,
      msf_tmp::CopyQBlocksFromAuxiliaryStatesToQ<StateSequence_T>(Qd));

  // TODO (slynen) Optim: Multiplication of F blockwise, using the fact that aux
  // states have no entries outside their block.
  state_new->P = Fd * state_old->P * Fd.transpose() + Qd;

  // Set time for best cov prop to now.
  time_P_propagated = state_new->time;

}

template<typename EKFState_T>
void MSF_Core<EKFState_T>::Init(
    shared_ptr<MSF_MeasurementBase<EKFState_T> > measurement) {

  initialized_ = false;
  predictionMade_ = false;

  usleep(100000);  // Hack, Hack, Hack, Hack thread sync.

  MeasurementBuffer_.Clear();
  stateBuffer_.Clear();
  fuzzyTracker_.Reset();

  while (!queueFutureMeasurements_.empty())
    queueFutureMeasurements_.pop();

  // Push one state to the buffer to apply the init on.
  shared_ptr<EKFState_T> state(new EKFState_T);
  state->time = 0;  // Will be set by the measurement.

  // Reset new state to zero.
  boost::fusion::for_each(state->statevars, msf_tmp::ResetState());

  // Set intialial covariance for core states.
  SetPCore(state->P);

  // Apply init measurement, where the user can provide additional values for P.
  measurement->Apply(state, *this);

  // Hack: Wait for the external propagation to get the init message.
  usleep(10000);

  assert(state->time != 0);

  it_last_IMU = stateBuffer_.Insert(state);

  // Will be set upon first IMU message.
  time_P_propagated = state->time;

  MSF_INFO_STREAM("Initializing msf_core (built: " <<__DATE__<<")");

  // Echo params.
  MSF_INFO_STREAM(
      "Core parameters: "<<std::endl << "\tfixed_bias:\t" <<
      usercalc_.GetParamFixedBias() << std::endl << "\tfuzzythres:\t" <<
      usercalc_.GetParamFuzzyTrackingThreshold() << std::endl <<
      "\tnoise_acc:\t" << usercalc_.GetParamNoiseAcc() << std::endl <<
      "\tnoise_accbias:\t" << usercalc_.GetParamNoiseAccbias() << std::endl <<
      "\tnoise_gyr:\t" << usercalc_.GetParamNoiseGyr() << std::endl <<
      "\tnoise_gyrbias:\t" << usercalc_.GetParamNoiseGyrbias() << std::endl);


  MSF_INFO_STREAM("Core init with state: " << std::endl << state->Print());
  initialized_ = true;

  msf_timing::Timing::Print(std::cout);
}

template<typename EKFState_T>
void MSF_Core<EKFState_T>::AddMeasurement(
    shared_ptr<MSF_MeasurementBase<EKFState_T> > measurement) {

  // Return if not initialized of no imu data available.
  if (!initialized_ || !predictionMade_)
    return;

  // Check if the measurement is in the future where we don't have imu
  // measurements yet.
  if (measurement->time > stateBuffer_.GetLast()->time) {
    queueFutureMeasurements_.push(measurement);
    return;

  }
  // Check if there is still a state in the buffer for this message (too old).
  if (measurement->time < stateBuffer_.GetFirst()->time) {
    MSF_WARN_STREAM(
        "You tried to give me a measurement which is too far in the past. Are "
        "you sure your clocks are synced and delays compensated correctly? "
        "[measurement: "<<timehuman(measurement->time)<<" (s) first state in "
            "buffer: "<<timehuman(stateBuffer_.GetFirst()->time)<<" (s)]");
    return;  // Reject measurements too far in the past.
  }

  // Add this measurement to the buffer and get an iterator to it.
  typename measurementBufferT::iterator_T it_meas =
      MeasurementBuffer_.Insert(measurement);
  // Get an iterator the the end of the measurement buffer.
  typename measurementBufferT::iterator_T it_meas_end = MeasurementBuffer_
      .GetIteratorEnd();

  // No propagation if no update is applied.
  typename StateBuffer_T::iterator_T it_curr = stateBuffer_.GetIteratorEnd();

  bool appliedOne = false;

  isfuzzyState_ = false;

  // Now go through all the measurements and apply them one by one.
  for (; it_meas != it_meas_end; ++it_meas) {

    if (it_meas->second->time <= 0)  // Valid?
      continue;
    msf_timing::DebugTimer timer_meas_get_state("Get state for measurement");
    // Propagates covariance to state.
    shared_ptr<EKFState_T> state = GetClosestState(it_meas->second->time);
    timer_meas_get_state.Stop();
    if (state->time <= 0) {
      MSF_ERROR_STREAM_THROTTLE(
          1, __FUNCTION__<< " GetClosestState returned an invalid state");
      continue;
    }

    msf_timing::DebugTimer timer_meas_apply("Apply measurement");
    // Calls back core::ApplyCorrection(), which sets time_P_propagated to meas
    // time.
    it_meas->second->Apply(state, *this);
    timer_meas_apply.Stop();
    // Make sure to propagate to next measurement or up to now if no more
    // measurements. Propagate from current state.
    it_curr = stateBuffer_.GetIteratorAtValue(state);

    typename StateBuffer_T::iterator_T it_end;
    typename measurementBufferT::iterator_T it_nextmeas = it_meas;
    ++it_nextmeas;  // The next measurement in the list.
    // That was the last measurement, so propagate state to now.
    if (it_nextmeas == it_meas_end) {
      it_end = stateBuffer_.GetIteratorEnd();
    } else {
      it_end = stateBuffer_.GetIteratorClosestAfter(it_nextmeas->second->time);
      if (it_end != stateBuffer_.GetIteratorEnd()) {
        // Propagate to state closest after the measurement so we can interpolate.
        ++it_end;
      }
    }

    typename StateBuffer_T::iterator_T it_next = it_curr;
    ++it_next;

    msf_timing::DebugTimer timer_prop_state_after_meas(
        "Repropagate state to now");
    // Propagate to selected state.
    for (; it_curr != it_end && it_next != it_end &&
           it_curr->second->time != constants::INVALID_TIME &&
           it_next->second->time != constants::INVALID_TIME; ++it_curr,
           ++it_next) {
      if (it_curr->second == it_next->second) {
        MSF_ERROR_STREAM(__FUNCTION__<< " propagation : it_curr points to same "
        "state as it_next. This must not happen.");
        continue;
      }
      // Break loop if EKF reset in the meantime.
      if (!initialized_ || !predictionMade_)
        return;
      PropagateState(it_curr->second, it_next->second);
    }
    timer_prop_state_after_meas.Stop();
    appliedOne = true;
  }

  if (!appliedOne) {
    MSF_ERROR_STREAM("No measurement was applied, this should not happen.");
    return;
  }

  // Now publish the best current estimate.
  shared_ptr<EKFState_T>& latestState = stateBuffer_.GetLast();

  PropPToState(latestState);  // Get the latest covariance.

  usercalc_.PublishStateAfterUpdate(latestState);
}

template<typename EKFState_T>
shared_ptr<msf_core::MSF_MeasurementBase<EKFState_T> >
MSF_Core<EKFState_T>::GetPreviousMeasurement(
    double time, int sensorID) {
  typename measurementBufferT::iterator_T it = MeasurementBuffer_
      .GetIteratorAtValue(time);
  if (it->second->time != time) {
    MSF_WARN_STREAM("GetPreviousMeasurement: Error invalid iterator at value");
    return MeasurementBuffer_.GetInvalid();
  }
  --it;
  typename measurementBufferT::iterator_T itbeforebegin = MeasurementBuffer_
      .GetIteratorBeforeBegin();
  while (it != itbeforebegin
      && (it->second->time != time && it->second->sensorID_ != sensorID)) {
    --it;
  }
  if (it == itbeforebegin) {
    MSF_WARN_STREAM("GetPreviousMeasurement: Error hit before begin");
    return MeasurementBuffer_.GetInvalid();
  }
  return it->second;
}

template<typename EKFState_T>
shared_ptr<EKFState_T> MSF_Core<EKFState_T>::GetStateAtTime(double tstamp) {
  return stateBuffer_.GetValueAt(tstamp);
}

template<typename EKFState_T>
shared_ptr<EKFState_T> MSF_Core<EKFState_T>::GetClosestState(double tstamp) {

  double timenow = tstamp;  // Delay compensated by sensor handler.

  typename StateBuffer_T::iterator_T it = stateBuffer_.GetIteratorClosest(
      timenow);

  shared_ptr<EKFState_T> closestState = it->second;
  // Check if the state really is close to the requested time.
  // With the new buffer this might not be given.
  if (closestState->time == constants::INVALID_TIME
      || fabs(closestState->time - timenow) > 0.1) {
    MSF_ERROR_STREAM(
        __FUNCTION__<< " Requested closest state to "<<timehuman(timenow)<<" but "
        "there was no suitable state in the map");
    // Not enough predictions made yet to apply measurement (too far in past).
    return stateBuffer_.GetInvalid();
  }

  // Do state interpolation if state is too far away from the measurement.
  double tdiff = fabs(closestState->time - timenow);  // Timediff to closest state.
  // If time diff too large, insert new state and do state interpolation.
  if (tdiff > 0.001) {
    shared_ptr<EKFState_T> lastState = stateBuffer_.GetClosestBefore(timenow);
    shared_ptr<EKFState_T> nextState = stateBuffer_.GetClosestAfter(timenow);

    bool statevalid = lastState->time != constants::INVALID_TIME
        && nextState->time != constants::INVALID_TIME;
    bool statenotnan = lastState->CheckStateForNumeric()
        && nextState->CheckStateForNumeric();
    bool statesnotsame = lastState->time != nextState->time;

    // If one of the states is invalid, we don't do interpolation, but just
    // take closest.
    if (statevalid && statenotnan && statesnotsame) {

      // Prepare a new state.
      shared_ptr<EKFState_T> currentState(new EKFState_T);
      currentState->time = timenow;  // Set state time to measurement time.
      // Linearly interpolate imu readings.
      currentState->a_m = lastState->a_m
          + (nextState->a_m - lastState->a_m)
              / (nextState->time - lastState->time)
              * (timenow - lastState->time);
      currentState->w_m = lastState->w_m
          + (nextState->w_m - lastState->w_m)
              / (nextState->time - lastState->time)
              * (timenow - lastState->time);

      currentState->noise_gyr = Vector3::Constant(usercalc_.GetParamNoiseGyr());
      currentState->noise_acc = Vector3::Constant(usercalc_.GetParamNoiseAcc());

      // Propagate with respective dt.
      PropagateState(lastState, currentState);

      stateBuffer_.Insert(currentState);

      // Make sure we propagate P correctly to the new state.
      if (time_P_propagated > lastState->time) {
        time_P_propagated = lastState->time;
      }

      closestState = currentState;
    }
  }
  // Catch up with covariance propagation if necessary.
  PropPToState(closestState);

  if (!closestState->CheckStateForNumeric()) {
    MSF_ERROR_STREAM(
        __FUNCTION__<< " State interpolation: interpolated state is invalid (nan)");
    return stateBuffer_.GetInvalid();  // Early abort.
  }

  static int janitorRun = 0;
  if (janitorRun++ > 100) {
    // Remove very old states and measurements from the buffers.
    CleanUpBuffers();
    janitorRun = 0;
  }
  return closestState;
}

template<typename EKFState_T>
void MSF_Core<EKFState_T>::PropPToState(shared_ptr<EKFState_T>& state) {
  // Propagate cov matrix until the current states time.
  typename StateBuffer_T::iterator_T it = stateBuffer_.GetIteratorAtValue(
      time_P_propagated, false);
  typename StateBuffer_T::iterator_T itMinus = it;
  ++it;
  // Until we reached the current state or the end of the state list.
  for (; it != stateBuffer_.GetIteratorEnd() && it->second->time <= state->time;
      ++it, ++itMinus) {
    PredictProcessCovariance(itMinus->second, it->second);
  }
}

template<typename EKFState_T>
bool MSF_Core<EKFState_T>::ApplyCorrection(shared_ptr<EKFState_T>& delaystate,
                                           ErrorState & correction,
                                           double fuzzythres) {

  if (!initialized_ || !predictionMade_)
    return false;

  // Give the user the possibility to fix some states.
  usercalc_.AugmentCorrectionVector(correction);

  // Now augment core states.
  if (usercalc_.GetParamFixedBias()) {
    typedef typename msf_tmp::GetEnumStateType<StateSequence_T,
        StateDefinition_T::b_a>::value b_a_type;
    typedef typename msf_tmp::GetEnumStateType<StateSequence_T,
        StateDefinition_T::b_w>::value b_w_type;

    enum {
      indexOfState_b_a = msf_tmp::GetStartIndex<StateSequence_T, b_a_type,
          msf_tmp::CorrectionStateLengthForType>::value,
      indexOfState_b_w = msf_tmp::GetStartIndex<StateSequence_T, b_w_type,
          msf_tmp::CorrectionStateLengthForType>::value
    };

    static_assert(
        static_cast<int>(indexOfState_b_w)==9,
        "The index of the state b_w in the correction vector differs from the "
        "expected value");
    static_assert(
        static_cast<int>(indexOfState_b_a)==12,
        "The index of the state b_a in the correction vector differs from the "
        "expected value");

    for (int i = 0;
        i < msf_tmp::StripConstReference<b_a_type>::result_t::sizeInCorrection_;
        ++i) {
      correction(indexOfState_b_a + i) = 0;  // Acc bias x,y,z.
    }
    for (int i = 0;
        i < msf_tmp::StripConstReference<b_w_type>::result_t::sizeInCorrection_;
        ++i) {
      correction(indexOfState_b_w + i) = 0;  // Gyro bias x,y,z.
    }
  }

  // State update:
  // TODO(slynen) What to do with attitude? Augment measurement noise?
  // Store old values in case of fuzzy tracking.
  EKFState_T buffstate = *delaystate;

  // Call correction function for every state.
  delaystate->Correct(correction);

  // Allow the user to sanity check the new state.
  usercalc_.SanityCheckCorrection(*delaystate, buffstate, correction);

  // TODO(slynen): Allow multiple fuzzy tracking states at the same time.
  isfuzzyState_ |= fuzzyTracker_.Check(delaystate, buffstate, fuzzythres);

  // No publishing and propagation here, because this might not be the last
  // update we have to apply.
  CheckForNumeric(correction, "update");

  // Set time latest propagated, we need to repropagate at least from here.
  time_P_propagated = delaystate->time;

  return 1;
}
}  // namespace msf_core
#endif  // MSF_CORE_INL_H_
