/*
 * Copyright (C) 2012-2013 Simon Lynen, ASL, ETH Zurich, Switzerland
 * You can contact the author at <slynen at ethz dot ch>
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
#ifndef MSF_STATE_INL_H_
#define MSF_STATE_INL_H_

#include <msf_core/msf_fwds.h>
#include <msf_core/msf_types.h>
#include <msf_core/msf_tmp.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <glog/logging.h>
#include <vector>
#include <msf_core/eigen_conversions.h>
#include <nav_msgs/Odometry.h>
#include <sensor_fusion_comm/ExtState.h>
#include <sensor_fusion_comm/DoubleArrayStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <boost/lexical_cast.hpp>

namespace msf_core {

// Returns the stateVar at position INDEX in the state list, non const version
// only for msf_core use you must not make these functions public. Instead
// const_cast the state object to const to use the overload.
template<typename stateVector_T, typename StateDefinition_T>
template<int INDEX>
inline typename boost::fusion::result_of::at_c<stateVector_T, INDEX>::type
GenericState_T<stateVector_T, StateDefinition_T>::GetStateVariable() {

  static_assert(
      (msf_tmp::IsReferenceType<typename
          boost::fusion::result_of::at_c<stateVector_T, INDEX >::type>::value),
      "Assumed that boost::fusion would return a reference type here, which is not the case.");

  return boost::fusion::at < boost::mpl::int_<INDEX> > (statevars);
}

// Returns the state at position INDEX in the state list, non const version
// you must not make these functions public. Instead const_cast the state object
// to const to use the overload.
template<typename stateVector_T, typename StateDefinition_T>
template<int INDEX>
inline typename msf_tmp::StripReference<
    typename boost::fusion::result_of::at_c<stateVector_T, INDEX>::type>::result_t::value_t&
GenericState_T<stateVector_T, StateDefinition_T>::Get() {
  static_assert(
      (msf_tmp::IsReferenceType<typename
          boost::fusion::result_of::at_c<stateVector_T, INDEX >::type>::value),
      "Assumed that boost::fusion would return a reference type here, which is not the case.");

  return boost::fusion::at < boost::mpl::int_<INDEX> > (statevars).state_;
}

// Apply the correction vector to all state vars.
template<typename stateVector_T, typename StateDefinition_T>
inline void GenericState_T<stateVector_T, StateDefinition_T>::Correct(
    const Eigen::Matrix<double, nErrorStatesAtCompileTime, 1>& correction) {
  boost::fusion::for_each(
      statevars,
      msf_tmp::CorrectState<
          const Eigen::Matrix<double, nErrorStatesAtCompileTime, 1>,
          stateVector_T>(correction));
}

// Returns the Q-block of the state at position INDEX in the state list, not
// allowed for core states.
template<typename stateVector_T, typename StateDefinition_T>
template<int INDEX>
inline typename msf_tmp::StripReference<
    typename boost::fusion::result_of::at_c<stateVector_T, INDEX>::type>::result_t::Q_T&
GenericState_T<stateVector_T, StateDefinition_T>::GetQBlock() {
  typedef typename msf_tmp::StripReference<
      typename boost::fusion::result_of::at_c<stateVector_T, INDEX>::type>::result_t StateVar_T;

  static_assert(
      static_cast<int>(StateVar_T::statetype_) != static_cast<int>(
          msf_core::CoreStateWithPropagation) ||
      static_cast<int>(StateVar_T::statetype_) !=
      static_cast<int>(msf_core::CoreStateWithoutPropagation),
      "You requested a non-const reference to a Q-Block for a"
      "core state of the EKF, but this is not allowed! Use the const version to "
      "get Q-blocks for core states.");

  return boost::fusion::at < boost::mpl::int_<INDEX> > (statevars).Q;
}

// Returns the Q-block of the state at position INDEX in the state list, also
// possible for core states, since const.
template<typename stateVector_T, typename StateDefinition_T>
template<int INDEX>
inline const typename msf_tmp::StripReference<
    typename boost::fusion::result_of::at_c<stateVector_T, INDEX>::type>::result_t::Q_T&
GenericState_T<stateVector_T, StateDefinition_T>::GetQBlock() const {
  return boost::fusion::at < boost::mpl::int_<INDEX> > (statevars).Q_;
}

/// Assembles a PoseWithCovarianceStamped message from the state
/** it does not set the header */
template<typename stateVector_T, typename StateDefinition_T>
void GenericState_T<stateVector_T, StateDefinition_T>::ToPoseMsg(
    geometry_msgs::PoseWithCovarianceStamped & pose) {
  eigen_conversions::Vector3dToPoint(Get<StateDefinition_T::p>(),
                                     pose.pose.pose.position);
  eigen_conversions::QuaternionToMsg(Get<StateDefinition_T::q>(),
                                     pose.pose.pose.orientation);
  GetPoseCovariance(pose.pose.covariance);
}


/// Assembles an Odometry message from the state.
/** it does not set the header */
template<typename stateVector_T, typename StateDefinition_T>
void GenericState_T<stateVector_T, StateDefinition_T>::ToOdometryMsg(
    nav_msgs::Odometry& odometry) {
  eigen_conversions::Vector3dToPoint(Get<StateDefinition_T::p>(),
                                     odometry.pose.pose.position);
  eigen_conversions::QuaternionToMsg(Get<StateDefinition_T::q>(),
                                     odometry.pose.pose.orientation);
  GetPoseCovariance(odometry.pose.covariance);

  const msf_core::Quaternion& q_W_I = Get<StateDefinition_T::q>();

  // Express velocity in body frame as demanded by the ROS odometry message specification.
  const msf_core::Vector3& v_W =  Get<StateDefinition_T::v>();
  const msf_core::Vector3 v_I = q_W_I.inverse().toRotationMatrix() * v_W;
  eigen_conversions::Vector3dToPoint(v_I,
                                     odometry.twist.twist.linear);

  // Subtract bias from gyro measurement.
  eigen_conversions::Vector3dToPoint(w_m - Get<StateDefinition_T::b_w>(),
                                     odometry.twist.twist.angular);

  GetTwistCovarianceInImuFrame(odometry.twist.covariance);
}

/// Assembles an ExtState message from the state
/** it does not set the header */
template<typename stateVector_T, typename StateDefinition_T>
void GenericState_T<stateVector_T, StateDefinition_T>::ToExtStateMsg(
    sensor_fusion_comm::ExtState & state) {
  eigen_conversions::Vector3dToPoint(Get<StateDefinition_T::p>(),
                                     state.pose.position);
  eigen_conversions::QuaternionToMsg(Get<StateDefinition_T::q>(),
                                     state.pose.orientation);
  eigen_conversions::Vector3dToPoint(Get<StateDefinition_T::v>(),
                                     state.velocity);
}

/// Assembles a DoubleArrayStamped message from the state
/** it does not set the header */
template<typename stateVector_T, typename StateDefinition_T>
void GenericState_T<stateVector_T, StateDefinition_T>::ToFullStateMsg(
    sensor_fusion_comm::DoubleArrayStamped & state) {
  state.data.resize(nStatesAtCompileTime);  // Make sure this is correctly sized.
  boost::fusion::for_each(
      statevars,
      msf_tmp::FullStatetoDoubleArray<std::vector<double>, stateVector_T>(
          state.data));
}

/// Assembles a DoubleArrayStamped message from the state
/** it does not set the header */
template<typename stateVector_T, typename StateDefinition_T>
void GenericState_T<stateVector_T, StateDefinition_T>::ToCoreStateMsg(
    sensor_fusion_comm::DoubleArrayStamped & state) {
  state.data.resize(nCoreStatesAtCompileTime);  // Make sure this is correctly sized.
  boost::fusion::for_each(
      statevars,
      msf_tmp::CoreStatetoDoubleArray<std::vector<double>, stateVector_T>(
          state.data));
}

template<typename stateVector_T, typename StateDefinition_T>
Eigen::Matrix<double,
    GenericState_T<stateVector_T,
                   StateDefinition_T>::nCoreStatesAtCompileTime,1>
GenericState_T<stateVector_T, StateDefinition_T>::ToEigenVector() {
  Eigen::Matrix<double,
      GenericState_T<stateVector_T, StateDefinition_T>::nCoreStatesAtCompileTime,
      1> data;
  boost::fusion::for_each(
      statevars,
      msf_tmp::CoreStatetoDoubleArray<
          typename Eigen::Matrix<double,
              GenericState_T<stateVector_T,
                             StateDefinition_T>::nCoreStatesAtCompileTime, 1>,
                             stateVector_T>(data));
  return data;
}

//TODO (slynen) Template to container.
template<typename stateVector_T, typename StateDefinition_T>
void GenericState_T<stateVector_T, StateDefinition_T>::CalculateIndicesInErrorState(
    std::vector<std::tuple<int, int, int> >& vec) {
  boost::fusion::for_each(
      statevars,
      msf_tmp::GetIndicesInErrorState<std::vector<std::tuple<int, int, int> >,
          stateVector_T>(vec));
}

template<typename stateVector_T, typename StateDefinition_T>
std::string GenericState_T<stateVector_T, StateDefinition_T>::Print() {
  std::stringstream ss;
  ss << "--------- State at time " << msf_core::timehuman(time)
      << "s: ---------" << std::endl;
  boost::fusion::for_each(
      statevars,
      msf_tmp::FullStatetoString<std::stringstream, stateVector_T>(ss));
  ss << "-------------------------------------------------------";
  return ss.str();
}

template<typename stateVector_T, typename StateDefinition_T>
bool GenericState_T<stateVector_T, StateDefinition_T>::CheckStateForNumeric() {
  Eigen::Matrix<double,
      GenericState_T<stateVector_T,
                     StateDefinition_T>::nCoreStatesAtCompileTime, 1> data;
  boost::fusion::for_each(
      statevars,
      msf_tmp::CoreStatetoDoubleArray<
          typename Eigen::Matrix<double,
              GenericState_T<stateVector_T, StateDefinition_T>::nCoreStatesAtCompileTime,
              1>, stateVector_T>(data));

  return CheckForNumeric(data, "CheckStateForNumeric");
}

// Returns the state at position INDEX in the state list, const version.
template<typename stateVector_T, typename StateDefinition_T>
template<int INDEX>
inline const typename msf_tmp::StripReference<
    typename boost::fusion::result_of::at_c<stateVector_T, INDEX>::type>::result_t::value_t&
GenericState_T<stateVector_T, StateDefinition_T>::Get() const {
  static_assert(
      (msf_tmp::IsReferenceType<typename boost::fusion::result_of::at_c<stateVector_T, INDEX >::type>::value),
      "Assumed that boost::fusion would return a reference type here, which is "
      "not the case");

  return boost::fusion::at < boost::mpl::int_<INDEX> > (statevars).state_;
}

template<typename stateVector_T, typename StateDefinition_T>
template<int INDEX>
inline typename msf_tmp::AddConstReference<
    typename boost::fusion::result_of::at_c<stateVector_T, INDEX>::type>::result_t
    GenericState_T<stateVector_T, StateDefinition_T>::GetStateVariable() const {
  static_assert(
      (msf_tmp::IsReferenceType<typename boost::fusion::result_of::at_c<stateVector_T, INDEX >::type>::value),
      "Assumed that boost::fusion would return a reference type here, which is "
      "not the case");
  return boost::fusion::at < boost::mpl::int_<INDEX> > (statevars);
}

template<typename stateVector_T, typename StateDefinition_T>
template<int INDEX>
inline void GenericState_T<stateVector_T, StateDefinition_T>::Set(
    const typename msf_tmp::StripConstReference<
        typename boost::fusion::result_of::at_c
            <stateVector_T, INDEX>::type>::result_t::value_t& newvalue) {
  typedef typename msf_tmp::StripReference<
      typename boost::fusion::result_of::at_c
          <stateVector_T, INDEX>::type>::result_t StateVar_T;

  static_assert(
      static_cast<int>(StateVar_T::statetype_) !=
      static_cast<int>(msf_core::CoreStateWithPropagation) ||
      static_cast<int>(StateVar_T::statetype_) !=
      static_cast<int>(msf_core::CoreStateWithoutPropagation),
      "You requested to set a new value for a"
      "core state of the EKF, but this is not allowed! This is an Error.");
  boost::fusion::at < boost::mpl::int_<INDEX> > (statevars).state_ = newvalue;
}

template<typename stateVector_T, typename StateDefinition_T>
template<int INDEX>
inline void GenericState_T<stateVector_T, StateDefinition_T>::ClearCrossCov() {
  typedef typename msf_tmp::StripReference<
      typename boost::fusion::result_of::at_c<stateVector_T, INDEX>::type>::result_t StateVar_T;

  enum {
    startIdxInState = msf_tmp::GetStartIndex<StateSequence_T, StateVar_T,
    // Index of the data in the correction vector.
        msf_tmp::CorrectionStateLengthForType>::value,
    lengthInState = StateVar_T::sizeInCorrection_
  };
  // Save covariance block.
  Eigen::Matrix<double, lengthInState, lengthInState> cov = P
      .template block<lengthInState, lengthInState>(startIdxInState,
                                                    startIdxInState);
  P.template block<lengthInState, nErrorStatesAtCompileTime>(startIdxInState, 0)
      .setZero();
  P.template block<nErrorStatesAtCompileTime, lengthInState>(0, startIdxInState)
      .setZero();
  // Write back cov block.
  P.template block<lengthInState, lengthInState>(startIdxInState,
                                                 startIdxInState) = cov;

}

/// Resets the state.
/**
 * 3D vectors: 0; quaternion: unit quaternion; scale: 1; time:0;
 * Error covariance: zeros
 */
template<typename stateVector_T, typename StateDefinition_T>
void GenericState_T<stateVector_T, StateDefinition_T>::Reset(
    msf_core::StateVisitor<GenericState_T<stateVector_T, StateDefinition_T> >* statevisitor) {

  // Reset all states.
  boost::fusion::for_each(statevars, msf_tmp::ResetState());

  // Reset system inputs.
  w_m.setZero();
  a_m.setZero();

  P.setZero();
  time = 0;

  // Now call the user provided function.
  if (statevisitor)
    statevisitor->ResetState(*this);
}

/// Writes the covariance corresponding to position and attitude to cov.
template<typename stateVector_T, typename StateDefinition_T>
void GenericState_T<stateVector_T, StateDefinition_T>::GetPoseCovariance(
    geometry_msgs::PoseWithCovariance::_covariance_type& cov) {

  typedef typename msf_tmp::GetEnumStateType<stateVector_T, StateDefinition_T::p>::value p_type;
  typedef typename msf_tmp::GetEnumStateType<stateVector_T, StateDefinition_T::q>::value q_type;

  // Get indices of position and attitude in the covariance matrix.
  static const int idxstartcorr_p = msf_tmp::GetStartIndex<stateVector_T,
      p_type, msf_tmp::CorrectionStateLengthForType>::value;
  static const int idxstartcorr_q = msf_tmp::GetStartIndex<stateVector_T,
      q_type, msf_tmp::CorrectionStateLengthForType>::value;

  /*        |  cov_p_p  |  cov_p_q  |
   *        |           |           |
   * cov =  |-----------|-----------|
   *        |           |           |
   *        |  cov_q_p  |  cov_q_q  |
   */

  CHECK_EQ(cov.size(), 6 * 6);
  Eigen::Map<Eigen::Matrix<double,6,6> > covariance_map(cov.data());

  covariance_map.block<3,3>(0,0) = P.template block<3,3>(idxstartcorr_p, idxstartcorr_p);
  covariance_map.block<3,3>(3,0) = P.template block<3,3>(idxstartcorr_q, idxstartcorr_p);
  covariance_map.block<3,3>(0,3) = P.template block<3,3>(idxstartcorr_p, idxstartcorr_q);
  covariance_map.block<3,3>(3,3) = P.template block<3,3>(idxstartcorr_q, idxstartcorr_q);
}

/// Writes the covariance corresponding to velocity and attitude to cov.
template<typename stateVector_T, typename StateDefinition_T>
void GenericState_T<stateVector_T, StateDefinition_T>::GetVelocityAttitudeCovariance(
    Eigen::Matrix<double, 6, 6>& cov) {

  typedef typename msf_tmp::GetEnumStateType<stateVector_T, StateDefinition_T::v>::value v_type;
  typedef typename msf_tmp::GetEnumStateType<stateVector_T, StateDefinition_T::q>::value q_type;

  // Get indices of position and attitude in the covariance matrix.
  static const int idxstartcorr_v = msf_tmp::GetStartIndex<stateVector_T,
      v_type, msf_tmp::CorrectionStateLengthForType>::value;
  static const int idxstartcorr_q = msf_tmp::GetStartIndex<stateVector_T,
      q_type, msf_tmp::CorrectionStateLengthForType>::value;

  /*        |  cov_v_v  |  cov_v_q  |
   *        |           |           |
   * cov =  |-----------|-----------|
   *        |           |           |
   *        |  cov_q_v  |  cov_q_q  |
   */

  cov.block<3,3>(0,0) = P.template block<3,3>(idxstartcorr_v, idxstartcorr_v);
  cov.block<3,3>(3,0) = P.template block<3,3>(idxstartcorr_q, idxstartcorr_v);
  cov.block<3,3>(0,3) = P.template block<3,3>(idxstartcorr_v, idxstartcorr_q);
  cov.block<3,3>(3,3) = P.template block<3,3>(idxstartcorr_q, idxstartcorr_q);
}

/// Writes the covariance corresponding to velocity and angular velocity expressed in the IMU frame
// to cov.
template<typename stateVector_T, typename StateDefinition_T>
void GenericState_T<stateVector_T, StateDefinition_T>::GetTwistCovarianceInImuFrame(
    geometry_msgs::TwistWithCovariance::_covariance_type& cov) {
  typedef typename msf_tmp::GetEnumStateType<stateVector_T, StateDefinition_T::b_w>::value b_w_type;

  // Get index of gyro bias in the covariance matrix.
  static const int idxstartcorr_b_w = msf_tmp::GetStartIndex<stateVector_T,
      b_w_type, msf_tmp::CorrectionStateLengthForType>::value;

  const msf_core::Quaternion& q_W_I = Get<StateDefinition_T::q>();
  const msf_core::Matrix3 R_W_I = q_W_I.toRotationMatrix();
  const msf_core::Vector3& v_W =  Get<StateDefinition_T::v>();
  const msf_core::Vector3 v_I =  R_W_I.transpose() * v_W;

  msf_core::Matrix6 cov_velocity_attitude_W;
  GetVelocityAttitudeCovariance(cov_velocity_attitude_W);

  Eigen::Matrix<double, 3, 6> J;
  J.block<3,3>(0,0) = R_W_I.transpose();
  J.block<3,3>(0,3) = Skew(v_I);

  // Transform noise covariance of velocity into the IMU frame.
  const msf_core::Matrix3 cov_velocity_I = J * cov_velocity_attitude_W * J.transpose();

  msf_core::Matrix3 cov_noise_gyr;
  cov_noise_gyr <<  noise_gyr[0] * noise_gyr[0], 0, 0,
                    0, noise_gyr[1] * noise_gyr[1], 0,
                    0, 0, noise_gyr[2] * noise_gyr[2];

  CHECK_EQ(cov.size(), 6 * 6);
  Eigen::Map<Eigen::Matrix<double,6,6> > covariance_map(cov.data());

  covariance_map.block<3,3>(0,0) = cov_velocity_I;

  // Add covariance of gyro measurement and gyro bias to get the covariance of the corrected
  // angular velocity.
  covariance_map.block<3,3>(3,3) = P.template block<3,3>(idxstartcorr_b_w, idxstartcorr_b_w) +
                                   cov_noise_gyr;
}

template<typename stateVector_T, typename StateDefinition_T>
void GenericState_T<stateVector_T, StateDefinition_T>::GetCoreCovariance(
    sensor_fusion_comm::DoubleMatrixStamped& cov) {

  const int n_core = nCoreErrorStatesAtCompileTime;
  cov.data.resize(n_core * n_core);
  cov.rows = n_core;
  cov.cols = n_core;

  for (int row = 0; row < n_core; ++row) {
    for (int col = 0; col < n_core; ++col) {
      cov.data[row * n_core + col] = P(row, col);
    }
  }
}

template<typename stateVector_T, typename StateDefinition_T>
void GenericState_T<stateVector_T, StateDefinition_T>::GetAuxCovariance(
    sensor_fusion_comm::DoubleMatrixStamped& cov) {

  const int n_core = nCoreErrorStatesAtCompileTime;
  const int n_aux = nErrorStatesAtCompileTime-n_core;
  cov.data.resize(n_aux * n_aux);
  cov.rows = n_aux;
  cov.cols = n_aux;

  int idx = 0;
  for (int row = n_core; row < nErrorStatesAtCompileTime; ++row) {
    for (int col = n_core; col < nErrorStatesAtCompileTime; ++col) {
      cov.data[idx] = P(row, col);
      ++idx;
    }
  }
}

template<typename stateVector_T, typename StateDefinition_T>
void GenericState_T<stateVector_T, StateDefinition_T>::GetCoreAuxCovariance(
    sensor_fusion_comm::DoubleMatrixStamped& cov) {

  const int n_core = nCoreErrorStatesAtCompileTime;
  const int n_aux = nErrorStatesAtCompileTime - n_core;
  cov.data.resize(n_core * n_aux);
  cov.rows = n_core;
  cov.cols = n_aux;

  // Use upper right block --> n_core rows and n_aux cols.
  int idx = 0;
  for (int row = 0; row < n_core; ++row) {
    for (int col = n_core; col < nErrorStatesAtCompileTime; ++col) {
      cov.data[idx] = P(row, col);
      ++idx;
    }
  }
}

}  // namespace msf_core
#endif  // MSF_STATE_INL_H_
