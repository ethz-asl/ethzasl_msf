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

#ifndef MEASUREMENT_H_
#define MEASUREMENT_H_

#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>

#include <msf_core/msf_fwds.h>
#include <msf_core/msf_types.tpp>

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
  MSF_MeasurementBase(bool isabsoluteMeasurement, int sensorID);
  virtual ~MSF_MeasurementBase() {
  }
  /**
   * \brief The method called by the msf_core to apply the measurement
   * represented by this object.
   */
  virtual void apply(shared_ptr<EKFState_T> stateWithCovariance,
                     MSF_Core<EKFState_T>& core) = 0;
  virtual std::string type() = 0;
  int sensorID_;
  bool isabsolute_;
  double time;  ///< The time_ this measurement was taken.
 protected:
  /**
   * Main update routine called by a given sensor, will apply the measurement to
   * the state inside the core.
   */
  template<class H_type, class Res_type, class R_type>
  void calculateAndApplyCorrection(
      shared_ptr<EKFState_T> state, MSF_Core<EKFState_T>& core,
      const Eigen::MatrixBase<H_type>& H_delayed,
      const Eigen::MatrixBase<Res_type> & res_delayed,
      const Eigen::MatrixBase<R_type>& R_delayed);

  void calculateAndApplyCorrection(shared_ptr<EKFState_T> state,
                                   MSF_Core<EKFState_T>& core,
                                   const Eigen::MatrixXd& H_delayed,
                                   const Eigen::MatrixXd & res_delayed,
                                   const Eigen::MatrixXd& R_delayed);

  template<class H_type, class Res_type, class R_type>
  void calculateAndApplyCorrectionRelative(
      shared_ptr<EKFState_T> state_old,
      shared_ptr<EKFState_T> state_new, MSF_Core<EKFState_T>& core,
      const Eigen::MatrixBase<H_type>& H_old,
      const Eigen::MatrixBase<H_type>& H_new,
      const Eigen::MatrixBase<Res_type> & res,
      const Eigen::MatrixBase<R_type>& R);

};

/**
 * \brief An invalid measurement needed for the measurement container to report
 * if something went wrong.
 */
template<typename EKFState_T>
class MSF_InvalidMeasurement : public MSF_MeasurementBase<EKFState_T> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  virtual void apply(
      shared_ptr<EKFState_T> UNUSEDPARAM(stateWithCovariance),
      MSF_Core<EKFState_T>& UNUSEDPARAM(core)) {
    MSF_ERROR_STREAM(
        "Called apply() on an MSF_InvalidMeasurement object. This should never "
        "happen.");
  }
  virtual std::string type() {
    return "invalid";
  }
  MSF_InvalidMeasurement()
      : MSF_MeasurementBase<EKFState_T>(true, -1) { }
  virtual ~MSF_InvalidMeasurement() { }
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
  virtual void makeFromSensorReadingImpl(
      const boost::shared_ptr<T const> reading) = 0;
 protected:
  RMAT_T R_;
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  typedef T Measurement_type;
  typedef boost::shared_ptr<T const> Measurement_ptr;

  MSF_Measurement(bool isAbsoluteMeasurement, int sensorID)
      : MSF_MeasurementBase<EKFState_T>(isAbsoluteMeasurement, sensorID) {
    R_.setZero();
  }
  virtual ~MSF_Measurement() {
  }
  ;
  void makeFromSensorReading(const boost::shared_ptr<T const> reading,
                             double timestamp) {
    this->time = timestamp;

    makeFromSensorReadingImpl(reading);

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
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MSF_InitMeasurement(bool ContainsInitialSensorReadings)
      : MSF_MeasurementBase<EKFState_T>(true, -1) {
    ContainsInitialSensorReadings_ = ContainsInitialSensorReadings;
    this->time = ros::Time::now().toSec();
  }
  virtual ~MSF_InitMeasurement() {
  }
  ;
  virtual std::string type() {
    return "init";
  }
  typename EKFState_T::P_type& get_P() {
    return InitState.P;
  }
  /**
   * \brief Get the gyro measurement.
   */
  Eigen::Matrix<double, 3, 1>& get_w_m() {
    return InitState.w_m;
  }
  /**
   * \brief Get the acceleration measurment.
   */
  Eigen::Matrix<double, 3, 1>& get_a_m() {
    return InitState.a_m;
  }

  /**
   * \brief Set the flag that the state variable at index INDEX has init values
   * for the state.
   */
  template<int INDEX, typename T>
  void setStateInitValue(const T& initvalue) {
    InitState.template getStateVar<INDEX>().state_ = initvalue;
    InitState.template getStateVar<INDEX>().hasResetValue = true;
  }

  /**
   * \brief Reset the flag that the state variable at index INDEX has init
   * values for the state.
   */
  template<int INDEX>
  void resetStateInitValue() {
    InitState.template get<INDEX>().hasResetValue = false;
  }

  /**
   * \brief Get the value stored in this object to initialize a state variable
   * at index INDEX.
   */
  template<int INDEX>
  const typename msf_tmp::StripReference<typename boost::fusion::result_of::at_c
      <StateSequence_T, INDEX >::type>::result_t::value_t&
  getStateInitValue() const {
    return InitState.template get<INDEX>();
  }

  virtual void apply(shared_ptr<EKFState_T> stateWithCovariance, MSF_Core<EKFState_T>& core);
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
                  const MSF_MeasurementBase<EKFState_T>&rhs) const {
    return (lhs.time_ < rhs.time_);
  }
};

}

#include <msf_core/implementation/msf_measurement.hpp>

#endif  // MEASUREMENT_H_
