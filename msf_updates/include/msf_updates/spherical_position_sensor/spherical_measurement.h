/*
 * Copyright (C) 2012-2013 Simon Lynen, ASL, ETH Zurich, Switzerland
 * You can contact the author at <slynen at ethz dot ch>
 * Copyright (C) 2013 Georg Wiedebach, ASL, ETH Zurich, Switzerland
 * You can contact the author at <georgwi at ethz dot ch>
 * Copyright (c) 2012, Markus Achtelik, ASL, ETH Zurich, Switzerland
 * You can contact the author at <acmarkus at ethz dot ch>
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

#ifndef SPHERICAL_POSITION_MEASUREMENT_HPP_
#define SPHERICAL_POSITION_MEASUREMENT_HPP_

#include <msf_core/msf_measurement.h>
#include <msf_core/msf_core.h>
#include <msf_core/eigen_utils.h>
#include <geometry_msgs/PointStamped.h>

namespace msf_spherical_position {
enum {
  N_ANGLE_MEASUREMENTS = 2,
  N_DISTANCE_MEASUREMENTS = 1
};

/**
 * \brief An angle measurement from a spherical position sensor.
 */
typedef msf_core::MSF_Measurement<geometry_msgs::PointStamped,
    Eigen::Matrix<double, N_ANGLE_MEASUREMENTS, N_ANGLE_MEASUREMENTS>,
    msf_updates::EKFState> AngleMeasurementBase;
struct AngleMeasurement : public AngleMeasurementBase {
 private:
  typedef AngleMeasurementBase Measurement_t;
  typedef Measurement_t::Measurement_ptr measptr_t;

  virtual void MakeFromSensorReadingImpl(measptr_t msg) {

    Eigen::Matrix<double, N_ANGLE_MEASUREMENTS,
        msf_core::MSF_Core<msf_updates::EKFState>::nErrorStatesAtCompileTime> H_old;
    Eigen::Matrix<double, N_ANGLE_MEASUREMENTS, 1> r_old;

    H_old.setZero();

    // Get measurement.
    z_a_ << msg->point.x, msg->point.y;

    if (fixed_covariance_) {   //  Take fix covariance from reconfigure GUI.
      const double s_zp = n_za_ * n_za_;
      R_ = (Eigen::Matrix<double, N_ANGLE_MEASUREMENTS, 1>() << s_zp, s_zp)
          .finished().asDiagonal();
    } else {   // Take covariance from sensor.
      ROS_WARN_STREAM_THROTTLE(
          60, "using non-fixed covariance not implemented yet");
      const double s_zp = n_za_ * n_za_;
      R_ = (Eigen::Matrix<double, N_ANGLE_MEASUREMENTS, 1>() << s_zp, s_zp)
          .finished().asDiagonal();
    }
  }
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  msf_core::Vector2 z_a_;  /// Position measurement.
  double n_za_;  /// Position measurement noise.

  bool fixed_covariance_;
  int fixedstates_;

  typedef msf_updates::EKFState EKFState_T;
  typedef EKFState_T::StateSequence_T StateSequence_T;
  typedef EKFState_T::StateDefinition_T StateDefinition_T;
  virtual ~AngleMeasurement() {
  }
  AngleMeasurement(double n_za, bool fixed_covariance,
                   bool isabsoluteMeasurement, int sensorID, int fixedstates,
                   bool enable_mah_outlier_rejection, double mah_threshold)
      : AngleMeasurementBase(isabsoluteMeasurement, sensorID,
                             enable_mah_outlier_rejection, mah_threshold),
        n_za_(n_za),
        fixed_covariance_(fixed_covariance),
        fixedstates_(fixedstates) {}
  virtual std::string Type() {
    return "angle";
  }

  virtual void CalculateH(
      boost::shared_ptr<EKFState_T> state_in,
      Eigen::Matrix<double, N_ANGLE_MEASUREMENTS,
          msf_core::MSF_Core<EKFState_T>::nErrorStatesAtCompileTime>& H) {
    const EKFState_T& state = *state_in;  // Get a const ref, so we can read core states.

    H.setZero();

    // Preprocess for elements in H matrix.
    // Get indices of states in error vector.
    enum {
      idxstartcorr_p_ = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
          StateDefinition_T::p>::value,
      idxstartcorr_v_ = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
          StateDefinition_T::v>::value,
      idxstartcorr_q_ = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
          StateDefinition_T::q>::value,
      idxstartcorr_p_pi_ = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
          StateDefinition_T::p_ip>::value,
    };

    bool fixed_p_pos_imu = (fixedstates_ & 1 << StateDefinition_T::p_ip);

    // Clear cross correlations.
    if (fixed_p_pos_imu)
      state_in->ClearCrossCov<StateDefinition_T::p_ip>();

    Eigen::Matrix<double, 3, 3> C_q = state.Get<StateDefinition_T::q>()
        .conjugate().toRotationMatrix();
    Eigen::Matrix<double, 3, 1> p_ = state.Get<StateDefinition_T::p>();
    Eigen::Matrix<double, 3, 1> p_ip = state.Get<StateDefinition_T::p_ip>();
    Eigen::Matrix<double, 2, 3> dz_dp;
    dz_dp(0, 0) = (p_(0, 0) * p_(2, 0) * 1.0
        / sqrt(p_(0, 0) * p_(0, 0) + p_(1, 0) * p_(1, 0)))
        / (p_(0, 0) * p_(0, 0) + p_(1, 0) * p_(1, 0) + p_(2, 0) * p_(2, 0));
    dz_dp(0, 1) = (p_(1, 0) * p_(2, 0) * 1.0
        / sqrt(p_(0, 0) * p_(0, 0) + p_(1, 0) * p_(1, 0)))
        / (p_(0, 0) * p_(0, 0) + p_(1, 0) * p_(1, 0) + p_(2, 0) * p_(2, 0));
    dz_dp(0, 2) = -sqrt(p_(0, 0) * p_(0, 0) + p_(1, 0) * p_(1, 0))
        / (p_(0, 0) * p_(0, 0) + p_(1, 0) * p_(1, 0) + p_(2, 0) * p_(2, 0));
    dz_dp(1, 0) = -p_(1, 0) / (p_(0, 0) * p_(0, 0) + p_(1, 0) * p_(1, 0));
    dz_dp(1, 1) = p_(0, 0) / (p_(0, 0) * p_(0, 0) + p_(1, 0) * p_(1, 0));
    dz_dp(1, 2) = 0;
    Eigen::Matrix<double, 2, 3> dz_dq;
    dz_dq(0, 0) = 1.0
        / sqrt(
            1.0
                / (p_(0, 0) * p_(0, 0) + p_(1, 0) * p_(1, 0)
                    + p_(2, 0) * p_(2, 0))) * 1.0
        / sqrt(p_(0, 0) * p_(0, 0) + p_(1, 0) * p_(1, 0)) * 1.0
        / pow(p_(0, 0) * p_(0, 0) + p_(1, 0) * p_(1, 0) + p_(2, 0) * p_(2, 0),
              3.0 / 2.0)
        * (C_q(2, 1) * p_ip(2, 0) * (p_(0, 0) * p_(0, 0))
            - C_q(2, 2) * p_ip(1, 0) * (p_(0, 0) * p_(0, 0))
            + C_q(2, 1) * p_ip(2, 0) * (p_(1, 0) * p_(1, 0))
            - C_q(2, 2) * p_ip(1, 0) * (p_(1, 0) * p_(1, 0))
            - C_q(0, 1) * p_ip(2, 0) * p_(0, 0) * p_(2, 0)
            + C_q(0, 2) * p_ip(1, 0) * p_(0, 0) * p_(2, 0)
            - C_q(1, 1) * p_ip(2, 0) * p_(1, 0) * p_(2, 0)
            + C_q(1, 2) * p_ip(1, 0) * p_(1, 0) * p_(2, 0));
    dz_dq(0, 1) = -1.0
        / sqrt(
            -(p_(2, 0) * p_(2, 0))
                / (p_(0, 0) * p_(0, 0) + p_(1, 0) * p_(1, 0)
                    + p_(2, 0) * p_(2, 0)) + 1.0) * 1.0
        / pow(p_(0, 0) * p_(0, 0) + p_(1, 0) * p_(1, 0) + p_(2, 0) * p_(2, 0),
              3.0 / 2.0)
        * (C_q(2, 0) * p_ip(2, 0) * (p_(0, 0) * p_(0, 0))
            - C_q(2, 2) * p_ip(0, 0) * (p_(0, 0) * p_(0, 0))
            + C_q(2, 0) * p_ip(2, 0) * (p_(1, 0) * p_(1, 0))
            - C_q(2, 2) * p_ip(0, 0) * (p_(1, 0) * p_(1, 0))
            - C_q(0, 0) * p_ip(2, 0) * p_(0, 0) * p_(2, 0)
            + C_q(0, 2) * p_ip(0, 0) * p_(0, 0) * p_(2, 0)
            - C_q(1, 0) * p_ip(2, 0) * p_(1, 0) * p_(2, 0)
            + C_q(1, 2) * p_ip(0, 0) * p_(1, 0) * p_(2, 0));
    dz_dq(0, 2) = 1.0
        / sqrt(
            1.0
                / (p_(0, 0) * p_(0, 0) + p_(1, 0) * p_(1, 0)
                    + p_(2, 0) * p_(2, 0))) * 1.0
        / sqrt(p_(0, 0) * p_(0, 0) + p_(1, 0) * p_(1, 0)) * 1.0
        / pow(p_(0, 0) * p_(0, 0) + p_(1, 0) * p_(1, 0) + p_(2, 0) * p_(2, 0),
              3.0 / 2.0)
        * (C_q(2, 0) * p_ip(1, 0) * (p_(0, 0) * p_(0, 0))
            - C_q(2, 1) * p_ip(0, 0) * (p_(0, 0) * p_(0, 0))
            + C_q(2, 0) * p_ip(1, 0) * (p_(1, 0) * p_(1, 0))
            - C_q(2, 1) * p_ip(0, 0) * (p_(1, 0) * p_(1, 0))
            - C_q(0, 0) * p_ip(1, 0) * p_(0, 0) * p_(2, 0)
            + C_q(0, 1) * p_ip(0, 0) * p_(0, 0) * p_(2, 0)
            - C_q(1, 0) * p_ip(1, 0) * p_(1, 0) * p_(2, 0)
            + C_q(1, 1) * p_ip(0, 0) * p_(1, 0) * p_(2, 0));
    dz_dq(1, 0) = (p_(1, 0) * (C_q(0, 1) * p_ip(2, 0) - C_q(0, 2) * p_ip(1, 0))
        - p_(0, 0) * (C_q(1, 1) * p_ip(2, 0) - C_q(1, 2) * p_ip(1, 0)))
        / (p_(0, 0) * p_(0, 0) + p_(1, 0) * p_(1, 0));
    dz_dq(1, 1) = -(p_(1, 0) * (C_q(0, 0) * p_ip(2, 0) - C_q(0, 2) * p_ip(0, 0))
        - p_(0, 0) * (C_q(1, 0) * p_ip(2, 0) - C_q(1, 2) * p_ip(0, 0)))
        / (p_(0, 0) * p_(0, 0) + p_(1, 0) * p_(1, 0));
    dz_dq(1, 2) = (p_(1, 0) * (C_q(0, 0) * p_ip(1, 0) - C_q(0, 1) * p_ip(0, 0))
        - p_(0, 0) * (C_q(1, 0) * p_ip(1, 0) - C_q(1, 1) * p_ip(0, 0)))
        / (p_(0, 0) * p_(0, 0) + p_(1, 0) * p_(1, 0));
    Eigen::Matrix<double, 2, 3> dz_dp_ip;
    dz_dp_ip(0, 0) =
        -1.0
            / sqrt(
                -(p_(2, 0) * p_(2, 0))
                    / (p_(0, 0) * p_(0, 0) + p_(1, 0) * p_(1, 0)
                        + p_(2, 0) * p_(2, 0)) + 1.0) * 1.0
            / pow(
                p_(0, 0) * p_(0, 0) + p_(1, 0) * p_(1, 0) + p_(2, 0) * p_(2, 0),
                3.0 / 2.0)
            * (C_q(2, 0) * (p_(0, 0) * p_(0, 0))
                + C_q(2, 0) * (p_(1, 0) * p_(1, 0))
                - C_q(0, 0) * p_(0, 0) * p_(2, 0)
                - C_q(1, 0) * p_(1, 0) * p_(2, 0));
    dz_dp_ip(0, 1) =
        -1.0
            / sqrt(
                -(p_(2, 0) * p_(2, 0))
                    / (p_(0, 0) * p_(0, 0) + p_(1, 0) * p_(1, 0)
                        + p_(2, 0) * p_(2, 0)) + 1.0) * 1.0
            / pow(
                p_(0, 0) * p_(0, 0) + p_(1, 0) * p_(1, 0) + p_(2, 0) * p_(2, 0),
                3.0 / 2.0)
            * (C_q(2, 1) * (p_(0, 0) * p_(0, 0))
                + C_q(2, 1) * (p_(1, 0) * p_(1, 0))
                - C_q(0, 1) * p_(0, 0) * p_(2, 0)
                - C_q(1, 1) * p_(1, 0) * p_(2, 0));
    dz_dp_ip(0, 2) =
        -1.0
            / sqrt(
                -(p_(2, 0) * p_(2, 0))
                    / (p_(0, 0) * p_(0, 0) + p_(1, 0) * p_(1, 0)
                        + p_(2, 0) * p_(2, 0)) + 1.0) * 1.0
            / pow(
                p_(0, 0) * p_(0, 0) + p_(1, 0) * p_(1, 0) + p_(2, 0) * p_(2, 0),
                3.0 / 2.0)
            * (C_q(2, 2) * (p_(0, 0) * p_(0, 0))
                + C_q(2, 2) * (p_(1, 0) * p_(1, 0))
                - C_q(0, 2) * p_(0, 0) * p_(2, 0)
                - C_q(1, 2) * p_(1, 0) * p_(2, 0));
    dz_dp_ip(1, 0) = -(C_q(0, 0) * p_(1, 0) - C_q(1, 0) * p_(0, 0))
        / (p_(0, 0) * p_(0, 0) + p_(1, 0) * p_(1, 0));
    dz_dp_ip(1, 1) = -(C_q(0, 1) * p_(1, 0) - C_q(1, 1) * p_(0, 0))
        / (p_(0, 0) * p_(0, 0) + p_(1, 0) * p_(1, 0));
    dz_dp_ip(1, 2) = -(C_q(0, 2) * p_(1, 0) - C_q(1, 2) * p_(0, 0))
        / (p_(0, 0) * p_(0, 0) + p_(1, 0) * p_(1, 0));
    H.block<2, 3>(0, idxstartcorr_p_) = dz_dp;
    H.block<2, 3>(0, idxstartcorr_q_) = dz_dq;
    H.block<2, 3>(0, idxstartcorr_p_pi_) = dz_dp_ip;

  }

  /**
   * The method called by the msf_core to apply the measurement represented by
   * this object.
   */
  virtual void Apply(boost::shared_ptr<EKFState_T> state_nonconst_new,
                     msf_core::MSF_Core<EKFState_T>& core) {
    if (isabsolute_) {  // Does this measurement refer to an absolute measurement,
      // or is is just relative to the last measurement.
      const EKFState_T& state = *state_nonconst_new;  // Get a const ref, so we can read core states.
      // Init variables.
      Eigen::Matrix<double, N_ANGLE_MEASUREMENTS,
          msf_core::MSF_Core<EKFState_T>::nErrorStatesAtCompileTime> H_new;
      Eigen::Matrix<double, N_ANGLE_MEASUREMENTS, 1> r_old;

      CalculateH(state_nonconst_new, H_new);

      // Get rotation matrices.
      Eigen::Matrix<double, 3, 3> C_q = state.Get<StateDefinition_T::q>()
          .conjugate().toRotationMatrix();

      // Construct residuals.
      Eigen::Matrix<double, 3, 1> z_carth = (state.Get<StateDefinition_T::p>()
          + C_q.transpose() * state.Get<StateDefinition_T::p_ip>());
      double radius_old = sqrt(
          z_carth(0, 0) * z_carth(0, 0) + z_carth(1, 0) * z_carth(1, 0)
              + z_carth(2, 0) * z_carth(2, 0));
      double theta_old = acos(z_carth(2, 0) / radius_old);
      double phi_old = atan(z_carth(1, 0) / z_carth(0, 0));
      // Handle all exeptions that occur when transforming in spherical coordinates.
      if (z_carth(0, 0) < 0 && z_carth(1, 0) < 0) {
        phi_old -= M_PI;
      }
      if (z_carth(0, 0) < 0 && z_carth(1, 0) > 0) {
        phi_old += M_PI;
      }

      msf_core::Vector2 z_spherical;
      z_spherical << theta_old, phi_old;
      r_old = z_a_ - z_spherical;
      if (r_old(1, 0) < -M_PI) {
        r_old(1, 0) += 2 * M_PI;
      }
      if (r_old(1, 0) > M_PI) {
        r_old(1, 0) -= 2 * M_PI;
      }

      if (!CheckForNumeric(r_old, "r_old")) {
        ROS_ERROR_STREAM("r_old: " << r_old);
        ROS_WARN_STREAM(
            "state: "
                << const_cast<EKFState_T&>(state).ToEigenVector().transpose());
      }
      if (!CheckForNumeric(H_new, "H_old")) {
        ROS_ERROR_STREAM("H_old: " << H_new);
        ROS_WARN_STREAM(
            "state: "
                << const_cast<EKFState_T&>(state).ToEigenVector().transpose());
      }
      if (!CheckForNumeric(R_, "R_")) {
        ROS_ERROR_STREAM("R_: " << R_);
        ROS_WARN_STREAM(
            "state: "
                << const_cast<EKFState_T&>(state).ToEigenVector().transpose());
      }

      // Call update step in base class.
      this->CalculateAndApplyCorrection(state_nonconst_new, core, H_new, r_old,
                                        R_);

    } else {
      ROS_ERROR_STREAM_THROTTLE(
          1,
          "You chose to apply the position measurement as a relative quantitiy, "
          "which is currently not implemented.");
    }
  }
};

/**
 * \brief A distance measurement from a spherical position sensor.
 */
typedef msf_core::MSF_Measurement<geometry_msgs::PointStamped,
    Eigen::Matrix<double, N_DISTANCE_MEASUREMENTS, N_DISTANCE_MEASUREMENTS>,
    msf_updates::EKFState> DistanceMeasurementBase;
struct DistanceMeasurement : public DistanceMeasurementBase {
 private:
  typedef DistanceMeasurementBase Measurement_t;
  typedef Measurement_t::Measurement_ptr measptr_t;

  virtual void MakeFromSensorReadingImpl(measptr_t msg) {
    Eigen::Matrix<double, N_DISTANCE_MEASUREMENTS,
        msf_core::MSF_Core<msf_updates::EKFState>::nErrorStatesAtCompileTime> H_old;
    Eigen::Matrix<double, N_DISTANCE_MEASUREMENTS, 1> r_old;

    H_old.setZero();

    // Get measurement.
    z_d_ << msg->point.z;

    if (fixed_covariance_)  //  Take fix covariance from reconfigure GUI.
    {
      R_(0, 0) = n_zd_ * n_zd_;
    } else {  // Take covariance from sensor.
      R_(0, 0) = n_zd_ * n_zd_;
      ROS_WARN_STREAM_THROTTLE(
          60, "using non-fixed covariance not implemented yet");
    }
  }
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  msf_core::Vector1 z_d_;  /// Position measurement.
  double n_zd_;  /// Position measurement noise.

  bool fixed_covariance_;
  int fixedstates_;

  typedef msf_updates::EKFState EKFState_T;
  typedef EKFState_T::StateSequence_T StateSequence_T;
  typedef EKFState_T::StateDefinition_T StateDefinition_T;
  virtual ~DistanceMeasurement() {
  }
  DistanceMeasurement(double n_zd, bool fixed_covariance,
                      bool isabsoluteMeasurement, int sensorID, int fixedstates,
                      bool enable_mah_outlier_rejection, double mah_threshold)
      : DistanceMeasurementBase(isabsoluteMeasurement, sensorID,
                                enable_mah_outlier_rejection, mah_threshold),
        n_zd_(n_zd),
        fixed_covariance_(fixed_covariance),
        fixedstates_(fixedstates) {}
  virtual std::string Type() {
    return "distance";
  }

  virtual void CalculateH(
      boost::shared_ptr<EKFState_T> state_in,
      Eigen::Matrix<double, N_DISTANCE_MEASUREMENTS,
          msf_core::MSF_Core<EKFState_T>::nErrorStatesAtCompileTime>& H) {
    const EKFState_T& state = *state_in;  // Get a const ref, so we can read core states.

    H.setZero();

    // Get rotation matrices.
    Eigen::Matrix<double, 3, 3> C_q = state.Get<StateDefinition_T::q>()
        .conjugate().toRotationMatrix();

    // Preprocess for elements in H matrix.
    // Get indices of states in error vector.
    enum {
      idxstartcorr_p_ = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
          StateDefinition_T::p>::value,
      idxstartcorr_v_ = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
          StateDefinition_T::v>::value,
      idxstartcorr_q_ = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
          StateDefinition_T::q>::value,
      idxstartcorr_p_pi_ = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
          StateDefinition_T::p_ip>::value,
    };

    bool fixed_p_pos_imu = (fixedstates_ & 1 << StateDefinition_T::p_ip);

    // Clear cross correlations.
    if (fixed_p_pos_imu)
      state_in->ClearCrossCov<StateDefinition_T::p_ip>();

    Eigen::Matrix<double, 3, 1> p_ = state.Get<StateDefinition_T::p>();
    Eigen::Matrix<double, 3, 1> p_ip = state.Get<StateDefinition_T::p_ip>();
    Eigen::Matrix<double, 1, 3> dz_dp;
    dz_dp(0, 0) = p_(0, 0) * 1.0
        / sqrt(p_(0, 0) * p_(0, 0) + p_(1, 0) * p_(1, 0) + p_(2, 0) * p_(2, 0));
    dz_dp(0, 1) = p_(1, 0) * 1.0
        / sqrt(p_(0, 0) * p_(0, 0) + p_(1, 0) * p_(1, 0) + p_(2, 0) * p_(2, 0));
    dz_dp(0, 2) = p_(2, 0) * 1.0
        / sqrt(p_(0, 0) * p_(0, 0) + p_(1, 0) * p_(1, 0) + p_(2, 0) * p_(2, 0));
    Eigen::Matrix<double, 1, 3> dz_dq;
    dz_dq(0, 0) = (p_(0, 0)
        * (C_q(0, 1) * p_ip(2, 0) * 2.0 - C_q(0, 2) * p_ip(1, 0) * 2.0)
        + p_(1, 0)
            * (C_q(1, 1) * p_ip(2, 0) * 2.0 - C_q(1, 2) * p_ip(1, 0) * 2.0)
        + p_(2, 0)
            * (C_q(2, 1) * p_ip(2, 0) * 2.0 - C_q(2, 2) * p_ip(1, 0) * 2.0))
        * 1.0
        / sqrt(p_(0, 0) * p_(0, 0) + p_(1, 0) * p_(1, 0) + p_(2, 0) * p_(2, 0))
        * (-1.0 / 2.0);
    dz_dq(0, 1) = (p_(0, 0)
        * (C_q(0, 0) * p_ip(2, 0) * 2.0 - C_q(0, 2) * p_ip(0, 0) * 2.0)
        + p_(1, 0)
            * (C_q(1, 0) * p_ip(2, 0) * 2.0 - C_q(1, 2) * p_ip(0, 0) * 2.0)
        + p_(2, 0)
            * (C_q(2, 0) * p_ip(2, 0) * 2.0 - C_q(2, 2) * p_ip(0, 0) * 2.0))
        * 1.0
        / sqrt(p_(0, 0) * p_(0, 0) + p_(1, 0) * p_(1, 0) + p_(2, 0) * p_(2, 0))
        * (1.0 / 2.0);
    dz_dq(0, 2) = (p_(0, 0)
        * (C_q(0, 0) * p_ip(1, 0) * 2.0 - C_q(0, 1) * p_ip(0, 0) * 2.0)
        + p_(1, 0)
            * (C_q(1, 0) * p_ip(1, 0) * 2.0 - C_q(1, 1) * p_ip(0, 0) * 2.0)
        + p_(2, 0)
            * (C_q(2, 0) * p_ip(1, 0) * 2.0 - C_q(2, 1) * p_ip(0, 0) * 2.0))
        * 1.0
        / sqrt(p_(0, 0) * p_(0, 0) + p_(1, 0) * p_(1, 0) + p_(2, 0) * p_(2, 0))
        * (-1.0 / 2.0);
    Eigen::Matrix<double, 1, 3> dz_dp_ip;
    dz_dp_ip(0, 0) = (C_q(0, 0) * p_(0, 0) + C_q(1, 0) * p_(1, 0)
        + C_q(2, 0) * p_(2, 0)) * 1.0
        / sqrt(p_(0, 0) * p_(0, 0) + p_(1, 0) * p_(1, 0) + p_(2, 0) * p_(2, 0));
    dz_dp_ip(0, 1) = (C_q(0, 1) * p_(0, 0) + C_q(1, 1) * p_(1, 0)
        + C_q(2, 1) * p_(2, 0)) * 1.0
        / sqrt(p_(0, 0) * p_(0, 0) + p_(1, 0) * p_(1, 0) + p_(2, 0) * p_(2, 0));
    dz_dp_ip(0, 2) = (C_q(0, 2) * p_(0, 0) + C_q(1, 2) * p_(1, 0)
        + C_q(2, 2) * p_(2, 0)) * 1.0
        / sqrt(p_(0, 0) * p_(0, 0) + p_(1, 0) * p_(1, 0) + p_(2, 0) * p_(2, 0));
    H.block<1, 3>(0, idxstartcorr_p_) = dz_dp;
    H.block<1, 3>(0, idxstartcorr_q_) = dz_dq;
    H.block<1, 3>(0, idxstartcorr_p_pi_) = dz_dp_ip;
  }

  /**
   * The method called by the msf_core to apply the measurement represented by this object.
   */
  virtual void Apply(boost::shared_ptr<EKFState_T> state_nonconst_new,
                     msf_core::MSF_Core<EKFState_T>& core) {

    if (isabsolute_) {  // Does this measurement refer to an absolute measurement,
                        // or is is just relative to the last measurement
      const EKFState_T& state = *state_nonconst_new;  // Get a const ref, so we can read core states.
      // Init variables.
      Eigen::Matrix<double, N_DISTANCE_MEASUREMENTS,
          msf_core::MSF_Core<EKFState_T>::nErrorStatesAtCompileTime> H_new;
      Eigen::Matrix<double, N_DISTANCE_MEASUREMENTS, 1> r_old;

      CalculateH(state_nonconst_new, H_new);

      // Get rotation matrices.
      Eigen::Matrix<double, 3, 3> C_q = state.Get<StateDefinition_T::q>()
          .conjugate().toRotationMatrix();

      // Construct residuals
      Eigen::Matrix<double, 3, 1> z_carth = (state.Get<StateDefinition_T::p>()
          + C_q.transpose() * state.Get<StateDefinition_T::p_ip>());
      double radius_old = sqrt(
          z_carth(0, 0) * z_carth(0, 0) + z_carth(1, 0) * z_carth(1, 0)
              + z_carth(2, 0) * z_carth(2, 0));

      msf_core::Vector1 z_spherical;
      z_spherical << radius_old;
      r_old = z_d_ - z_spherical;

      if (!CheckForNumeric(r_old, "r_old")) {
        ROS_ERROR_STREAM("r_old: " << r_old);
        ROS_WARN_STREAM(
            "state: "
                << const_cast<EKFState_T&>(state).ToEigenVector().transpose());
      }
      if (!CheckForNumeric(H_new, "H_old")) {
        ROS_ERROR_STREAM("H_old: " << H_new);
        ROS_WARN_STREAM(
            "state: "
                << const_cast<EKFState_T&>(state).ToEigenVector().transpose());
      }
      if (!CheckForNumeric(R_, "R_")) {
        ROS_ERROR_STREAM("R_: " << R_);
        ROS_WARN_STREAM(
            "state: "
                << const_cast<EKFState_T&>(state).ToEigenVector().transpose());
      }

      // Call update step in base class.
      this->CalculateAndApplyCorrection(state_nonconst_new, core, H_new, r_old,
                                        R_);
    } else {
      ROS_ERROR_STREAM_THROTTLE(
          1,
          "You chose to apply the position measurement as a relative quantitiy, "
          "which is currently not implemented.");
    }
  }
};
}  // namespace msf_spherical_position
#endif  // SPHERICAL_POSITION_MEASUREMENT_HPP_
