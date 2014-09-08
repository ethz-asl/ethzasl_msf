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
 
/**
\mainpage ethzasl_msf

Time delay compensated single and multi sensor fusion framework based on an EKF. 
Please see the wiki for more information: https://github.com/ethz-asl/ethzasl_msf/wiki

The API is documented here: http://ethz-asl.github.io/ethzasl_msf

You are welcome contributing to the package by opening a pull-request: 
Please make yourself familiar with the Google c++ style guide: 
http://google-styleguide.googlecode.com/svn/trunk/cppguide.xml
*/ 
 
#ifndef MSF_CORE_H_
#define MSF_CORE_H_

#include <vector>
#include <queue>

#include <Eigen/Eigen>

#include <msf_core/msf_sortedContainer.h>
#include <msf_core/msf_state.h>
#include <msf_core/msf_checkFuzzyTracking.h>

namespace msf_core {
template<typename EKFState_T>
class MSF_SensorManager;
template<typename EKFState_T>
class IMUHandler;

/** \class MSF_Core
 *
 * \brief The core class of the EKF
 * Does propagation of state and covariance
 * but also applying measurements and managing states and measurements
 * in lists sorted by time stamp.
 */
template<typename EKFState_T>
class MSF_Core {
  friend class MSF_MeasurementBase<EKFState_T>;
  friend class IMUHandler<EKFState_T>;
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum {
    /// Error state length.
    nErrorStatesAtCompileTime = EKFState_T::nErrorStatesAtCompileTime,
    /// Complete state length.
    nStatesAtCompileTime = EKFState_T::nStatesAtCompileTime
  };

  typedef typename EKFState_T::StateDefinition_T StateDefinition_T;
  typedef typename EKFState_T::StateSequence_T StateSequence_T;
  /// The error state type.
  typedef Eigen::Matrix<double, nErrorStatesAtCompileTime, 1> ErrorState;
  /// The error state covariance type.
  typedef Eigen::Matrix<double, nErrorStatesAtCompileTime,
      nErrorStatesAtCompileTime> ErrorStateCov;

  /// The type of the state buffer containing all the states.
  typedef msf_core::SortedContainer<EKFState_T> StateBuffer_T;
  /// The type of the measurement buffer containing all the measurements
  typedef msf_core::SortedContainer<
      typename msf_core::MSF_MeasurementBase<EKFState_T>,
      typename msf_core::MSF_InvalidMeasurement<EKFState_T> > measurementBufferT;

  /**
   * \brief Add a sensor measurement or an init measurement to the internal
   * queue and apply it to the state.
   * \param Measurement the measurement to add to the internal measurement queue.
   */
  void AddMeasurement(shared_ptr<MSF_MeasurementBase<EKFState_T> > measurement);

  /**
   * \brief Initializes the filter with the values of the given measurement,
   * other init values from other sensors can be passed in as "measurement"
   * using the initMeasurement structs.
   * \param Measurement a measurement containing initial values for the state
   */
  void Init(shared_ptr<MSF_MeasurementBase<EKFState_T> > measurement);

  /**
   * \brief Finds the closest state to the requested time in the internal state.
   * \param tstamp The time stamp to find the closest state to.
   */
  shared_ptr<EKFState_T> GetClosestState(double tstamp);

  /**
   * \brief Returns the accumulated dynamic matrix between two states.
   */
  void GetAccumulatedStateTransitionStochasticCloning(
      const shared_ptr<EKFState_T>& state_old,
      const shared_ptr<EKFState_T>& state_new,
      Eigen::Matrix<double, nErrorStatesAtCompileTime, nErrorStatesAtCompileTime>& F);
  /**
   * \brief Returns previous measurement of the same type.
   */
  shared_ptr<msf_core::MSF_MeasurementBase<EKFState_T> > GetPreviousMeasurement(
      double time, int sensorID);

  /**
   * \brief Finds the state at the requested time in the internal state.
   * \param tstamp The time stamp to find the state to.
   */
  shared_ptr<EKFState_T> GetStateAtTime(double tstamp);

  /**
   * \brief Propagates the error state covariance.
   * \param state_old The state to propagate the covariance from.
   * \param state_new The state to propagate the covariance to.
   */
  void PredictProcessCovariance(shared_ptr<EKFState_T>& state_old,
                                shared_ptr<EKFState_T>& state_new);

  /**
   * \brief Propagates the state with given dt.
   * \param state_old The state to propagate from.
   * \param state_new The state to propagate to.
   */
  void PropagateState(shared_ptr<EKFState_T>& state_old,
                      shared_ptr<EKFState_T>& state_new);

  /**
   * \brief Delete very old states and measurements from the buffers to free
   * memory.
   */
  void CleanUpBuffers();

  /**
   * \brief sets the covariance matrix of the core states to simulated values.
   * \param P the error state covariance Matrix to fill.
   */
  void SetPCore(
      Eigen::Matrix<double, EKFState_T::nErrorStatesAtCompileTime,
          EKFState_T::nErrorStatesAtCompileTime>& P);

  /**
   * \brief Ctor takes a pointer to an object which does the user defined
   * calculations and provides interfaces for initialization etc.
   * \param GetUserCalc The class providing the user defined calculations
   * DO ABSOLUTELY NOT USE THIS REFERENCE INSIDE THIS CTOR!!
   */
  MSF_Core(const MSF_SensorManager<EKFState_T>& GetUserCalc);
  ~MSF_Core();

  const MSF_SensorManager<EKFState_T>& GetUserCalc() const;

 private:
  /**
   * \brief Get the index of the best state having no temporal drift at compile
   * time.
   */
  enum {
    indexOfStateWithoutTemporalDrift = msf_tmp::IndexOfBestNonTemporalDriftingState<
        StateSequence_T>::value
  };

  /// Returns void type for invalid types
  typedef typename msf_tmp::GetEnumStateType<StateSequence_T,
      indexOfStateWithoutTemporalDrift>::value nonDriftingStateType;

  /// EKF buffer containing pretty much all info needed at time t. Sorted by t
  // asc.
  StateBuffer_T stateBuffer_;
  /// EKF Measurements and init values sorted by t asc.
  measurementBufferT MeasurementBuffer_;
  /// Buffer for measurements to apply in future.
  std::queue<shared_ptr<MSF_MeasurementBase<EKFState_T> > > queueFutureMeasurements_;
  /// Last time stamp where we have a valid propagation.
  double time_P_propagated;
  /// Last time stamp where we have a valid state.
  typename StateBuffer_T::iterator_T it_last_IMU;

  /// Is the filter initialized, so that we can propagate the state?
  bool initialized_;
  /// Is there a state prediction, so we can apply measurements?
  bool predictionMade_;
  /// Was the filter pushed to fuzzy state by a measurement?
  bool isfuzzyState_;
  /// Watch dog to determine fuzzy tracking by observing non temporal drifting
  // states.
  CheckFuzzyTracking<EKFState_T, nonDriftingStateType> fuzzyTracker_;
  /// A class which provides methods for customization of several calculations.
  const MSF_SensorManager<EKFState_T>& usercalc_;

  /**
   * \brief Applies the correction.
   * \param delaystate The state to apply the correction on.
   * \param correction The correction vector.
   * \param fuzzythres The error of the non temporal drifting state allowed
   *  before fuzzy tracking will be triggered.
   */
  bool ApplyCorrection(shared_ptr<EKFState_T>& delaystate,
                       ErrorState & correction, double fuzzythres = 0.1);

  /**
   * \brief Propagate covariance to a given state in time.
   * \param State the state to propagate to from the last propagated time.
   */
  void PropPToState(shared_ptr<EKFState_T>& state);

  //Internal state propagation:
  /**
   * \brief This function gets called on incoming imu messages
   * and then performs the state prediction internally.
   * Only use this OR StateCallback by remapping the topics accordingly.
   * \param msg The imu ros message.
   * \sa{StateCallback}
   */
  void ProcessIMU(const msf_core::Vector3&linear_acceleration,
                   const msf_core::Vector3&angular_velocity,
                   const double& msg_stamp, size_t msg_seq);

  /// External state propagation:
  /**
   * \brief This function gets called when state prediction is performed
   * externally, e.g. by asctec_mav_framework. Msg has to be the latest
   * predicted state.
   * Only use this OR IMUCallback by remapping the topics accordingly.
   * \param msg The state message from the external propagation.
   * \sa{IMUCallback}
   */
  void ProcessExternallyPropagatedState(const msf_core::Vector3& linear_acceleration,
                        const msf_core::Vector3& angular_velocity,
                        const msf_core::Vector3& p, const msf_core::Vector3& v,
                        const msf_core::Quaternion& q,
                        bool is_already_propagated, const double& msg_stamp,
                        size_t msg_seq);

  /// Propagates P by one step to distribute processing load.
  void PropagatePOneStep();

  /// Checks the queue of measurements to be applied in the future.
  void HandlePendingMeasurements();
};
}
// msf_core

#include <msf_core/implementation/msf_core_inl.h>

#endif  // MSF_CORE_H_
