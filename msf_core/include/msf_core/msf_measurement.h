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
#ifndef MEASUREMENT_H_
#define MEASUREMENT_H_

#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>

#include <msf_core/msf_fwds.h>
#include <msf_core/msf_types.h>

namespace msf_core {
/**
 * \brief The base class for all measurement types.
 * These are the objects provided to the EKF core to be applied in correct order
 * to the states.
 */
template<typename EKFState_T>
class MSF_MeasurementBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MSF_MeasurementBase(bool isabsoluteMeasurement, int sensorID,
                      bool enable_mah_outlier_rejection,
                      double mah_threshold);
  virtual ~MSF_MeasurementBase() {}
  /**
   * \brief The method called by the msf_core to apply the measurement
   * represented by this object.
   */
  virtual void Apply(shared_ptr<EKFState_T> stateWithCovariance,
                     MSF_Core<EKFState_T>& core) = 0;
  virtual std::string Type() = 0;
  int sensorID_;
  bool isabsolute_;
  double time;  ///< The time_ this measurement was taken.
 protected:
  /**
   * Main update routine called by a given sensor, will apply the measurement to
   * the state inside the core.
   */
  template<class H_type, class Res_type, class R_type>
  void CalculateAndApplyCorrection(
      shared_ptr<EKFState_T> state, MSF_Core<EKFState_T>& core,
      const Eigen::MatrixBase<H_type>& H,
      const Eigen::MatrixBase<Res_type>& residual,
      const Eigen::MatrixBase<R_type>& R);

  void CalculateAndApplyCorrection(shared_ptr<EKFState_T> state,
                                   MSF_Core<EKFState_T>& core,
                                   const Eigen::MatrixXd& H,
                                   const Eigen::MatrixXd& residual,
                                   const Eigen::MatrixXd& R);

  template<class H_type, class Res_type, class R_type>
  void CalculateAndApplyCorrectionRelative(
      shared_ptr<EKFState_T> state_old, shared_ptr<EKFState_T> state_new,
      MSF_Core<EKFState_T>& core, const Eigen::MatrixBase<H_type>& H_old,
      const Eigen::MatrixBase<H_type>& H_new,
      const Eigen::MatrixBase<Res_type>& residual,
      const Eigen::MatrixBase<R_type>& R);

  /// Enables mahalanobis distance outlier rejection
  bool enable_mah_outlier_rejection_;

  /// Mahalanobis distance outlier rejection threshold
  double mah_threshold_;

};

/**
 * \brief An invalid measurement needed for the measurement container to report
 * if something went wrong.
 */
template<typename EKFState_T>
class MSF_InvalidMeasurement : public MSF_MeasurementBase<EKFState_T> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  virtual void Apply(shared_ptr<EKFState_T> UNUSEDPARAM(stateWithCovariance),
                     MSF_Core<EKFState_T>& UNUSEDPARAM(core)) {
    MSF_ERROR_STREAM(
        "Called Apply() on an MSF_InvalidMeasurement object. This should never "
        "happen.");
  }
  virtual std::string Type() {
    return "invalid";
  }
  MSF_InvalidMeasurement()
      : MSF_MeasurementBase<EKFState_T>(true, constants::INVALID_ID, false, 0.0) {
  }
  virtual ~MSF_InvalidMeasurement() {
  }
};

/**
 * \brief The class for sensor based measurements which we want to apply to
 * a state in the update routine of the EKF. This calls the apply correction
 * method of the EKF core.
 * \note Provides an abstract NVI to create measurements from sensor readings.
 */
template<typename T, typename RMAT_T, typename EKFState_T>
class MSF_Measurement : public MSF_MeasurementBase<EKFState_T> {
 private:
  virtual void MakeFromSensorReadingImpl(
      const boost::shared_ptr<T const> reading) = 0;
 protected:
  RMAT_T R_;
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef T Measurement_type;
  typedef boost::shared_ptr<T const> Measurement_ptr;

  MSF_Measurement(bool isAbsoluteMeasurement, int sensorID,
                  bool enable_mah_outlier_rejection, double mah_threshold)
      : MSF_MeasurementBase<EKFState_T>(isAbsoluteMeasurement, sensorID,
                                        enable_mah_outlier_rejection,
                                        mah_threshold) {
    R_.setZero();
  }
  virtual ~MSF_Measurement() { }
  void MakeFromSensorReading(const boost::shared_ptr<T const> reading,
                             double timestamp) {
    this->time = timestamp;
    MakeFromSensorReadingImpl(reading);

    // Check whether the user has set R.
    if (R_.minCoeff() == 0.0 && R_.maxCoeff() == 0.0) {
      MSF_WARN_STREAM_THROTTLE(
          2,
          "The measurement covariance matrix seems to be not set for the current "
          "measurement. Please double check!");
    }

    for (int i = 0; i < R_.RowsAtCompileTime; ++i) {
      if (R_(i, i) == 0.0) {
        MSF_WARN_STREAM_THROTTLE(
            2,
            "The measurement covariance matrix has some diagonal elements set to "
            "zero. Please double check!");
      }
    }
  }
// Apply is implemented by respective sensor measurement types.
};

/**
 * \brief A measurement to be send to initialize parts of or the full EKF state
 * this can especially be used to split the initialization of the EKF
 * between multiple sensors which init different parts of the state.
 */
template<typename EKFState_T>
class MSF_InitMeasurement : public MSF_MeasurementBase<EKFState_T> {
 private:
  EKFState_T InitState;  ///< Values for initialization of the state.
  /// Flag whether this measurement contains initial sensor readings.
  bool ContainsInitialSensorReadings_;
  typedef typename EKFState_T::StateSequence_T StateSequence_T;
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW MSF_InitMeasurement(
      bool ContainsInitialSensorReadings)
      : MSF_MeasurementBase<EKFState_T>(true, constants::INVALID_ID, false, 0.0) {
    ContainsInitialSensorReadings_ = ContainsInitialSensorReadings;
    this->time = ros::Time::now().toSec();
  }
  virtual ~MSF_InitMeasurement() {
  }
  ;
  virtual std::string Type() {
    return "init";
  }
  typename EKFState_T::P_type& GetStateCovariance() {
    return InitState.P;
  }
  /**
   * \brief Get the gyro measurement.
   */
  Eigen::Matrix<double, 3, 1>& Getw_m() {
    return InitState.w_m;
  }
  /**
   * \brief Get the acceleration measurment.
   */
  Eigen::Matrix<double, 3, 1>& Geta_m() {
    return InitState.a_m;
  }

  /**
   * \brief Set the flag that the state variable at index INDEX has init values
   * for the state.
   */
  template<int INDEX, typename T>
  void SetStateInitValue(const T& initvalue) {
    InitState.template GetStateVariable<INDEX>().state_ = initvalue;
    InitState.template GetStateVariable<INDEX>().hasResetValue = true;
  }

  /**
   * \brief Reset the flag that the state variable at index INDEX has init
   * values for the state.
   */
  template<int INDEX>
  void ResetStateInitValue() {
    InitState.template get<INDEX>().hasResetValue = false;
  }

  /**
   * \brief Get the value stored in this object to initialize a state variable
   * at index INDEX.
   */
  template<int INDEX>
  const typename msf_tmp::StripReference<typename boost::fusion::result_of::at_c
  <StateSequence_T, INDEX >::type>::result_t::value_t&
  GetStateInitValue() const {
    return InitState.template get<INDEX>();
  }

  virtual void Apply(shared_ptr<EKFState_T> stateWithCovariance, MSF_Core<EKFState_T>& core);
};

/**
* \brief A comparator to sort measurements by time.
*/
template<typename EKFState_T>
class sortMeasurements {
 public:
  /**
   * Implements the sorting by time.
   */
  bool operator()(const MSF_MeasurementBase<EKFState_T>& lhs,
                  const MSF_MeasurementBase<EKFState_T>& rhs) const {
    return (lhs.time_ < rhs.time_);
  }
};
}

#include <msf_core/implementation/msf_measurement_inl.h>

#endif  // MEASUREMENT_H_
