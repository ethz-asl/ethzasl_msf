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
#ifndef PRESSURE_MEASUREMENT_HPP_
#define PRESSURE_MEASUREMENT_HPP_

#include <msf_core/msf_measurement.h>
#include <msf_core/msf_core.h>

namespace pressure_measurement {
enum {
  nMeasurements = 1
};
/**
 * \brief A measurement as provided by a pressure sensor.
 */
typedef msf_core::MSF_Measurement<geometry_msgs::PointStamped,
    Eigen::Matrix<double, nMeasurements, nMeasurements>, msf_updates::EKFState> PressureMeasurementBase;
struct PressureMeasurement : public PressureMeasurementBase {
 private:
  typedef PressureMeasurementBase Measurement_t;
  typedef Measurement_t::Measurement_ptr measptr_t;

  virtual void MakeFromSensorReadingImpl(measptr_t msg) {
    Eigen::Matrix<double, nMeasurements,
        msf_core::MSF_Core<msf_updates::EKFState>::nErrorStatesAtCompileTime> H_old;
    Eigen::Matrix<double, nMeasurements, 1> r_old;

    H_old.setZero();

    // Get measurements.
    z_p_ = Eigen::Matrix<double, 1, 1>::Constant(msg->point.z);

    const double s_zp = n_zp_ * n_zp_;
    R_ = (Eigen::Matrix<double, nMeasurements, 1>() << s_zp).finished()
        .asDiagonal();
  }
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Matrix<double, 1, 1> z_p_;  /// Pressure measurement.
  double n_zp_;  /// Pressure measurement noise.

  typedef msf_updates::EKFState EKFState_T;
  typedef EKFState_T::StateDefinition_T StateDefinition_T;
  virtual ~PressureMeasurement() {}
  PressureMeasurement(double n_zp, bool isabsoluteMeasurement, int sensorID,
                      bool enable_mah_outlier_rejection, double mah_threshold)
      : PressureMeasurementBase(isabsoluteMeasurement, sensorID,
                                enable_mah_outlier_rejection, mah_threshold),
        n_zp_(n_zp) {}
  virtual std::string Type() { return "pressure"; }
  /**
   * The method called by the msf_core to apply the measurement represented by
   * this object.
   */
  virtual void Apply(shared_ptr<EKFState_T> non_const_state,
                     msf_core::MSF_Core<EKFState_T>& core) {
    // Init variables.
    Eigen::Matrix<double, nMeasurements,
        msf_core::MSF_Core<EKFState_T>::nErrorStatesAtCompileTime> H_old;
    Eigen::Matrix<double, nMeasurements, 1> r_old;

    H_old.setZero();

    if (non_const_state->time == msf_core::constants::INVALID_TIME) {
      MSF_WARN_STREAM(
          "Apply pressure update was called with an invalid state.");
      return;  // Early abort.
    }

    const EKFState_T& state = *non_const_state;

    enum {
      idx_p = msf_tmp::GetStartIndex<EKFState_T::StateSequence_T,
          typename msf_tmp::GetEnumStateType<EKFState_T::StateSequence_T,
              StateDefinition_T::p>::value,
          msf_tmp::CorrectionStateLengthForType>::value,

      idx_b_p = msf_tmp::GetStartIndex<EKFState_T::StateSequence_T,
          typename msf_tmp::GetEnumStateType<EKFState_T::StateSequence_T,
              StateDefinition_T::b_p>::value,
          msf_tmp::CorrectionStateLengthForType>::value
    };

    // Construct H matrix.
    // Position:
    H_old.block<1, 1>(0, idx_p + 2)(0) = 1;  // p_z
    // Pressure bias.
    H_old.block<1, 1>(0, idx_b_p)(0) = -1;  //p_b

    // Construct residuals.
    // Height.
    r_old.block<1, 1>(0, 0) = (z_p_ + state.Get<StateDefinition_T::b_p>())
        - state.Get<StateDefinition_T::p>().block<1, 1>(2, 0);

    // Call update step in base class.
    this->CalculateAndApplyCorrection(non_const_state, core, H_old, r_old, R_);
  }
};
}  // namespace pressure_measurement
#endif  // POSE_MEASUREMENT_HPP_
