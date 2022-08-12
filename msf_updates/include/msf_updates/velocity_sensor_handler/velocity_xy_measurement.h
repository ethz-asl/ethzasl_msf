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
  Eigen::Matrix<double, 2, 1> _z_v{
      Eigen::Matrix<double, 2, 1>::Zero()};  /// Velocity measurement in xy
                                             /// sensor coordinates.
  double _n_zv{0.0};                         /// Velocity measurement noise.

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
                        int fixedstates)
      : VelocityXYMeasurementBase(isabsoluteMeasurement, sensorID,
                                  enable_mah_outlier_rejection, mah_threshold),
        _n_zv(n_zv),
        _fixed_covariance(fixed_covariance),
        _fixedstates(fixedstates) {}

  virtual std::string Type() { return "velocity_xy"; }

  virtual void CalculateH(
      shared_ptr<EKFState_T> state_in,
      Eigen::Matrix<double, nMeasurements,
                    msf_core::MSF_Core<EKFState_T>::nErrorStatesAtCompileTime>&
          H) {
  }

  /**
   * The method called by the msf_core to apply the measurement represented by
   * this object.
   */
  virtual void Apply(shared_ptr<EKFState_T> state_nonconst_new,
                     msf_core::MSF_Core<EKFState_T>& core) {
    // TODO(clanegge): Complete Apply function
  }
};

}  // namespace velocity_xy_measurement
}  // namespace msf_updates

#endif  // VELOCITY_XY_MEASUREMENT_HPP_