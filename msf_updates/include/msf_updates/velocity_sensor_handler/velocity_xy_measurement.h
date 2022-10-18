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
#ifndef VELOCITY_XY_MEASUREMENT_HPP_
#define VELOCITY_XY_MEASUREMENT_HPP_

#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <msf_core/eigen_utils.h>
#include <msf_core/msf_core.h>
#include <msf_core/msf_measurement.h>

namespace msf_updates {
namespace velocity_xy_measurement {
enum { nMeasurements = 2 };

/**
 * \brief A 2D measurement as provided by an velocity sensor, e.g. optical flow.
 */
typedef msf_core::MSF_Measurement<
    geometry_msgs::TwistWithCovarianceStamped,
    Eigen::Matrix<double, nMeasurements, nMeasurements>, msf_updates::EKFState>
    VelocityXYMeasurementBase;

template <int StateQivIdx = EKFState::StateDefinition_T::q_iv,
          int StatePivIdx = EKFState::StateDefinition_T::p_iv>
struct VelocityXYMeasurement : public VelocityXYMeasurementBase {
 private:
  typedef VelocityXYMeasurementBase Measurement_t;
  typedef Measurement_t::Measurement_ptr measptr_t;

  const Eigen::Matrix<double, 3, 3> C_vi_{
      Eigen::Matrix<double, 3, 3>::Identity()};
  const Eigen::Vector3d piv_{Eigen::Vector3d::Zero()};
  const Eigen::Matrix<double, 3, 3> piv_sk_{
      Eigen::Matrix<double, 3, 3>::Zero()};

  virtual void MakeFromSensorReadingImpl(measptr_t msg) {
    Eigen::Matrix<
        double, nMeasurements,
        msf_core::MSF_Core<msf_updates::EKFState>::nErrorStatesAtCompileTime>
        H_old;
    Eigen::Matrix<double, nMeasurements, 1> r_old;

    H_old.setZero();

    // Get measurements
    _z_v = Eigen::Matrix<double, 2, 1>(msg->twist.twist.linear.x,
                                       msg->twist.twist.linear.y);

    // TODO(clanegge): implement
    if (!_fixed_covariance) {  // Take covariance from sensor.
      MSF_ERROR_STREAM(
          "Covariance from sensor not implemented yet. Using fixed "
          "covariance.");
    }

    // Take fix covariance from reconfigure GUI.
    const double s_zv = _n_zv * _n_zv;
    R_ = (Eigen::Matrix<double, nMeasurements, 1>() << s_zv, s_zv)
             .finished()
             .asDiagonal();

    // TODO(clanegge): DO we need to clear out cross-correlations?
    // std::cout << "Covariance matrix:\n" << R_ << std::endl;
    R_(0, 1) = 0.0;
    R_(1, 0) = 0.0;
  }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Matrix<double, nMeasurements, 1> _z_v{
      Eigen::Matrix<double, nMeasurements,
                    1>::Zero()};  /// Velocity measurement in xy
                                  /// sensor coordinates.
  double _n_zv{0.0};              /// Velocity measurement noise.

  bool _fixed_covariance{false};
  int _fixedstates{0};

  typedef msf_updates::EKFState EKFState_T;
  typedef EKFState_T::StateSequence_T StateSequence_T;
  typedef EKFState_T::StateDefinition_T StateDefinition_T;

  enum AuxState { q_iv = StateQivIdx, p_iv = StatePivIdx };

  virtual ~VelocityXYMeasurement() {}
  VelocityXYMeasurement(double n_zv, bool fixed_covariance,
                        bool isabsoluteMeasurement, int sensorID,
                        bool enable_mah_outlier_rejection, double mah_threshold,
                        int fixedstates,
                        const Eigen::Matrix<double, 3, 3>& C_vi,
                        const Eigen::Matrix<double, 3, 1>& p_iv)
      : VelocityXYMeasurementBase(isabsoluteMeasurement, sensorID,
                                  enable_mah_outlier_rejection, mah_threshold),
        C_vi_(C_vi),
        piv_(p_iv),
        piv_sk_(Skew(p_iv)),
        _n_zv(n_zv),
        _fixed_covariance(fixed_covariance),
        _fixedstates(fixedstates) {}

  virtual std::string Type() { return "velocity_xy"; }

  virtual void CalculateH(
      shared_ptr<EKFState_T> state_in,
      Eigen::Matrix<double, nMeasurements,
                    msf_core::MSF_Core<EKFState_T>::nErrorStatesAtCompileTime>&
          H) {
    const EKFState_T& state =
        *state_in;  // Get a const ref, so we can read core states.

    H.setZero();

    // Get rotation matrices.
    // Eigen::Matrix<double, 3, 3> C_vi =
    //     state.Get<StateQivIdx>().conjugate().toRotationMatrix();

    // Preprocess for elements in H matrix.
    // Eigen::Matrix<double, 3, 3> piv_sk = Skew(state.Get<StatePivIdx>());

    const Eigen::Matrix<double, 3, 3> C_I_W =
        state.Get<StateDefinition_T::q>().inverse().toRotationMatrix();

    // Get indices of states in error vector
    enum {
      kIdxstartcorr_v =
          msf_tmp::GetStartIndexInCorrection<StateSequence_T,
                                             StateDefinition_T::v>::value,
      kIdxstartcorr_q =
          msf_tmp::GetStartIndexInCorrection<StateSequence_T,
                                             StateDefinition_T::q>::value,
      kIdxstartcorr_bw =
          msf_tmp::GetStartIndexInCorrection<StateSequence_T,
                                             StateDefinition_T::b_w>::value,
    };

    // Read the fixed states flags.
    bool calibposfix = (_fixedstates & 1 << StatePivIdx);
    bool calibattfix = (_fixedstates & 1 << StateQivIdx);

    // TODO(clanegge) :
    if (!calibposfix || !calibattfix) {
      MSF_ERROR_STREAM(
          "Online calibration of velocity sensor not implemented yet. Using "
          "fixed transform.");
      calibposfix = true;
      calibattfix = true;
    }

    if (calibposfix) {
      state_in->ClearCrossCov<StatePivIdx>();
    }
    if (calibattfix) {
      state_in->ClearCrossCov<StateQivIdx>();
    }

    // Construct H matrix.
    // velocity:
    // C_vi * i_v_i
    H.block<2, 3>(0, kIdxstartcorr_v) =
        // C_vi_.block<2, 3>(0, 0);
        (C_vi_ * C_I_W).block<2, 3>(0, 0);
    // H.block<2, 3>(0, kIdxstartcorr_v) = C_vi.block<2, 3>(0, 0);

    // H.block<2, 3>(0, kIdxstartcorr_q) =
        (C_vi_ * Skew(C_I_W * state.Get<StateDefinition_T::v>()))
            .block<2, 3>(0, 0);

    // gyro bias/angular velocity:
    // Cross term C_vi*( [i_omega_i - i_b_w] x r_iv)
    // = C_vi*r_iv_skew*i_b_w
    H.block<2, 3>(0, kIdxstartcorr_bw) = (C_vi_ * piv_sk_).block<2, 3>(0, 0);
    // H.block<2, 3>(0, kIdxstartcorr_bw) = (C_vi * piv_sk).block<2, 3>(0, 0);
  }

  /**
   * The method called by the msf_core to apply the measurement represented by
   * this object.
   */
  virtual void Apply(shared_ptr<EKFState_T> state_nonconst_new,
                     msf_core::MSF_Core<EKFState_T>& core) {
    const EKFState_T& state = *state_nonconst_new;
    // init variables
    Eigen::Matrix<double, nMeasurements,
                  msf_core::MSF_Core<EKFState_T>::nErrorStatesAtCompileTime>
        H_new;
    Eigen::Matrix<double, nMeasurements, 1> r_old;
    CalculateH(state_nonconst_new, H_new);

    // Get rotation matrices.
    // Eigen::Matrix<double, 3, 3> C_vi =
    //     state.Get<StateQivIdx>().conjugate().toRotationMatrix();

    // Get body velocity and convert to imu frame
    const msf_core::Quaternion& q_W_I = state.Get<StateDefinition_T::q>();
    const msf_core::Vector3& v_W = state.Get<StateDefinition_T::v>();
    const msf_core::Vector3 v_I = q_W_I.inverse().toRotationMatrix() * v_W;

    // Construct residuals.
    // 3d Sensor velocity in imu frame:
    Eigen::Matrix<double, 3, 1> i_v_v =
        (v_I + (state.w_m - state.Get<StateDefinition_T::b_w>()).cross(piv_));
    // Eigen::Matrix<double, 3, 1> i_v_v =
    //     (state.Get<StateDefinition_T::v>() +
    //      (state.w_m - state.Get<StateDefinition_T::b_w>())
    //          .cross(state.Get<StatePivIdx>()));

    // We only measure x & y velocities - ignore z
    Eigen::Vector2d model_z = (C_vi_ * i_v_v).block<2, 1>(0, 0);
    MSF_WARN_STREAM("Measurement:\n" << _z_v);
    MSF_WARN_STREAM("Model:\n" << model_z);
    MSF_WARN_STREAM("State:\n" << state.Get<StateDefinition_T::v>());
    r_old = _z_v - (C_vi_ * i_v_v).block<2, 1>(0, 0);
    // r_old = _z_v - (C_vi * i_v_v).block<2, 1>(0, 0);

    if (!CheckForNumeric(r_old, "r_old")) {
      MSF_ERROR_STREAM("r_old: " << r_old);
      MSF_WARN_STREAM(
          "state: "
          << const_cast<EKFState_T&>(state).ToEigenVector().transpose());
    }
    if (!CheckForNumeric(H_new, "H_old")) {
      MSF_ERROR_STREAM("H_old: " << H_new);
      MSF_WARN_STREAM(
          "state: "
          << const_cast<EKFState_T&>(state).ToEigenVector().transpose());
    }
    if (!CheckForNumeric(R_, "R_")) {
      MSF_ERROR_STREAM("R_: " << R_);
      MSF_WARN_STREAM(
          "state: "
          << const_cast<EKFState_T&>(state).ToEigenVector().transpose());
    }

    // Call update step in base class.
    this->CalculateAndApplyCorrection(state_nonconst_new, core, H_new, r_old,
                                      R_);
  }
};

}  // namespace velocity_xy_measurement
}  // namespace msf_updates

#endif  // VELOCITY_XY_MEASUREMENT_HPP_