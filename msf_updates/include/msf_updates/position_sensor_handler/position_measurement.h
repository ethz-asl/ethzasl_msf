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
#ifndef POSITION_MEASUREMENT_HPP_
#define POSITION_MEASUREMENT_HPP_

#include <msf_core/msf_measurement.h>
#include <msf_core/msf_core.h>
#include <msf_core/eigen_utils.h>
#include <sensor_fusion_comm/PointWithCovarianceStamped.h>

namespace msf_updates {
namespace position_measurement {
enum {
  nMeasurements = 3
};

/**
 * \brief A measurement as provided by a position sensor, e.g. Total Station, GPS.
 */
typedef msf_core::MSF_Measurement<
    sensor_fusion_comm::PointWithCovarianceStamped,
    Eigen::Matrix<double, nMeasurements, nMeasurements>, msf_updates::EKFState> PositionMeasurementBase;
struct PositionMeasurement : public PositionMeasurementBase {
 private:
  typedef PositionMeasurementBase Measurement_t;
  typedef Measurement_t::Measurement_ptr measptr_t;

  virtual void MakeFromSensorReadingImpl(measptr_t msg) {

    Eigen::Matrix<double, nMeasurements,
        msf_core::MSF_Core<msf_updates::EKFState>::nErrorStatesAtCompileTime> H_old;
    Eigen::Matrix<double, nMeasurements, 1> r_old;

    H_old.setZero();

    // Get measurement.
    z_p_ = Eigen::Matrix<double, 3, 1>(msg->point.x, msg->point.y,
                                       msg->point.z);

    if (fixed_covariance_)  //  take fix covariance from reconfigure GUI
    {

      const double s_zp = n_zp_ * n_zp_;
      R_ = (Eigen::Matrix<double, nMeasurements, 1>() << s_zp, s_zp, s_zp)
          .finished().asDiagonal();

    } else {  // Tke covariance from sensor.

      R_.block<3, 3>(0, 0) = msf_core::Matrix3(&msg->covariance[0]);

      if (msg->header.seq % 100 == 0) {  // Only do this check from time to time.
        if (R_.block<3, 3>(0, 0).determinant() < -0.01)
          MSF_WARN_STREAM_THROTTLE(
              60, "The covariance matrix you provided for "
              "the position sensor is not positive definite");
      }
    }
  }
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Matrix<double, 3, 1> z_p_;  /// Position measurement.
  double n_zp_;  /// Position measurement noise.

  bool fixed_covariance_;
  int fixedstates_;

  typedef msf_updates::EKFState EKFState_T;
  typedef EKFState_T::StateSequence_T StateSequence_T;
  typedef EKFState_T::StateDefinition_T StateDefinition_T;
  virtual ~PositionMeasurement() {
  }
  PositionMeasurement(double n_zp, bool fixed_covariance,
                      bool isabsoluteMeasurement, int sensorID, int fixedstates,
                      bool enable_mah_outlier_rejection, double* mah_threshold,
                      double mah_rejection_modification, double mah_acceptance_modification, double mah_threshold_limit,
                      double* n_rejected, double* n_curr_rejected, double* n_accepted)
      : PositionMeasurementBase(isabsoluteMeasurement, sensorID,
                                enable_mah_outlier_rejection, mah_threshold,
                                mah_rejection_modification, mah_acceptance_modification, mah_threshold_limit,
                                n_rejected, n_curr_rejected, n_accepted),
        n_zp_(n_zp),
        fixed_covariance_(fixed_covariance),
        fixedstates_(fixedstates) {
  }
  virtual std::string Type() {
    return "position";
  }

  virtual void CalculateH(
      shared_ptr<EKFState_T> state_in,
      Eigen::Matrix<double, nMeasurements,
          msf_core::MSF_Core<EKFState_T>::nErrorStatesAtCompileTime>& H) {
    const EKFState_T& state = *state_in;  // Get a const ref, so we can read core states.

    H.setZero();

    // Get rotation matrices.
    Eigen::Matrix<double, 3, 3> C_q = state.Get<StateDefinition_T::q>()
        .conjugate().toRotationMatrix();

    // Preprocess for elements in H matrix.
    Eigen::Matrix<double, 3, 3> p_prism_imu_sk = Skew(
        state.Get<StateDefinition_T::p_ip>());

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

    // Clear crosscorrelations.
    if (fixed_p_pos_imu)
      state_in->ClearCrossCov<StateDefinition_T::p_ip>();

    // Construct H matrix:
    // Position:
    H.block<3, 3>(0, idxstartcorr_p_) = Eigen::Matrix<double, 3, 3>::Identity();  // p

    H.block<3, 3>(0, idxstartcorr_q_) = -C_q.transpose() * p_prism_imu_sk;  // q

    H.block<3, 3>(0, idxstartcorr_p_pi_) =
        fixed_p_pos_imu ?
            Eigen::Matrix<double, 3, 3>::Zero() : (C_q.transpose()).eval();  //p_pos_imu_

  }
  
  /**
   * this method add description (overriding base)
   */
  /*template<class H_type, class Res_type, class R_type>
  void CalculateAndApplyCorrection(
    shared_ptr<EKFState_T> state, msf_core::MSF_Core<EKFState_T>& core,
    const Eigen::MatrixBase<H_type>& H_delayed,
    const Eigen::MatrixBase<Res_type> & res_delayed,
    const Eigen::MatrixBase<R_type>& R_delayed) {

  EIGEN_STATIC_ASSERT_FIXED_SIZE (H_type);
  EIGEN_STATIC_ASSERT_FIXED_SIZE (R_type);

  // Get measurements.
  /// Correction from EKF update.
  Eigen::Matrix<double, msf_core::MSF_Core<EKFState_T>::nErrorStatesAtCompileTime, 1> correction_;

  R_type S;
  R_type S_inverse;
  Eigen::Matrix<double, msf_core::MSF_Core<EKFState_T>::nErrorStatesAtCompileTime,
      R_type::RowsAtCompileTime> K;
  typename msf_core::MSF_Core<EKFState_T>::ErrorStateCov & P = state->P;


  S = H_delayed * P * H_delayed.transpose() + R_delayed;
  S_inverse = S.inverse();
  
  MSF_WARN_STREAM("overriding for Position with mahtreshold:"<<mah_threshold_);
  
  if(enable_mah_outlier_rejection_){ //could do this earlier to save computation time
	  //MSF_WARN_STREAM("outlier rejection active");
    //calculate mahalanobis distance
    //Eigen::MatrixXd mah_dist_squared_temp = res_delayed.transpose() * S_inverse * res_delayed;//is this correct (this should output a scalar right?)
    double mah_dist_squared=res_delayed.transpose() * S_inverse * res_delayed;
    ros::NodeHandle pnh("~/position_sensor");
    
    //double mah_dist_squared = mah_dist_squared_temp(0,0);

    //reject point as outlier if distance above threshold
    //if (sqrt(mah_dist_squared) > mah_threshold_){ //should not compute sqrt for efficiency
    if(mah_dist_squared>mah_threshold_*mah_threshold_){
	  //mah_threshold_*=2;
	  //MSF_WARN_STREAM("new mah_threshold"<<mah_threshold_);
      MSF_WARN_STREAM("rejecting reading as outlier with distance squared"<<mah_dist_squared);
      pnh.setParam("mah_threshold", 2*mah_threshold_);
      return;
    }
    pnh.setParam("mah_threshold", 0.9*mah_threshold_);
  }
  //mah_threshold_*=1.2;
  K = P * H_delayed.transpose() * S_inverse;
  correction_ = K * res_delayed;
  const typename msf_core::MSF_Core<EKFState_T>::ErrorStateCov KH =
      (msf_core::MSF_Core<EKFState_T>::ErrorStateCov::Identity() - K * H_delayed);
  P = KH * P * KH.transpose() + K * R_delayed * K.transpose();

  // Make sure P stays symmetric.
  P = 0.5 * (P + P.transpose());

  core.ApplyCorrection(state, correction_);
} */

  /**
   * The method called by the msf_core to apply the measurement represented by this object.
   */
  virtual void Apply(shared_ptr<EKFState_T> state_nonconst_new,
                     msf_core::MSF_Core<EKFState_T>& core) {

    if (isabsolute_) {  // Does this measurement refer to an absolute measurement,
      // or is it relative to the last measurement.
      // Get a const ref, so we can read core states.
      const EKFState_T& state = *state_nonconst_new;
      // init variables
      Eigen::Matrix<double, nMeasurements,
          msf_core::MSF_Core<EKFState_T>::nErrorStatesAtCompileTime> H_new;
      Eigen::Matrix<double, nMeasurements, 1> r_old;

      CalculateH(state_nonconst_new, H_new);

      // Get rotation matrices.
      Eigen::Matrix<double, 3, 3> C_q = state.Get<StateDefinition_T::q>()
          .conjugate().toRotationMatrix();

      // Construct residuals:
      // Position
      r_old.block<3, 1>(0, 0) = z_p_
          - (state.Get<StateDefinition_T::p>()
              + C_q.transpose() * state.Get<StateDefinition_T::p_ip>());
              //here residual is computed seems ok'ish

      if (!CheckForNumeric(r_old, "r_old")) {
        MSF_ERROR_STREAM("r_old: "<<r_old);
        MSF_WARN_STREAM(
            "state: "<<const_cast<EKFState_T&>(state). ToEigenVector().transpose());
      }
      if (!CheckForNumeric(H_new, "H_old")) {
        MSF_ERROR_STREAM("H_old: "<<H_new);
        MSF_WARN_STREAM(
            "state: "<<const_cast<EKFState_T&>(state). ToEigenVector().transpose());
      }
      if (!CheckForNumeric(R_, "R_")) {
        MSF_ERROR_STREAM("R_: "<<R_);
        MSF_WARN_STREAM(
            "state: "<<const_cast<EKFState_T&>(state). ToEigenVector().transpose());
      }

      // Call update step in base class.
      this->CalculateAndApplyCorrection(state_nonconst_new, core, H_new, r_old,
                                        R_);
    } else {
      MSF_ERROR_STREAM_THROTTLE(
          1, "You chose to apply the position measurement "
          "as a relative quantitiy, which is currently not implemented.");
    }
  }
};
}  // namespace position_measurement
}  // namespace msf_updates

#endif  // POSITION_MEASUREMENT_HPP_
