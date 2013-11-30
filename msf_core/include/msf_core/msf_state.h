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
#ifndef MSF_STATE_H_
#define MSF_STATE_H_

#include <msf_core/msf_types.hpp>
#include <msf_core/msf_tmp.h>
#include <msf_core/msf_statevisitor.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <utility>
#include <msf_core/eigen_conversions.h>
#include <sensor_fusion_comm/ExtState.h>
#include <sensor_fusion_comm/DoubleArrayStamped.h>
#include <sensor_fusion_comm/DoubleMatrixStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace msf_core {

/**
 * \brief A state variable with a name as specified in the state name enum.
 */
template<typename type_T, int name_T, int STATETYPE, int OPTIONS>
struct StateVar_T {
  typedef type_T value_t;
  typedef const StateVar_T<type_T, name_T>& constRef_T;
  typedef const StateVar_T<type_T, name_T>* constPtr_T;
  typedef StateVar_T<type_T, name_T>& Ref_T;
  typedef StateVar_T<type_T, name_T>* Ptr_T;
  enum {
    ///The type of this state. needed for computations of total state length.
    statetype_ = STATETYPE,
    ///Option flags for this state variable.
    options_ = OPTIONS,
    ///The name of the state, needed to find it in the state type list.
    name_ = name_T,
    ///The size of this state in the correction vector.
    sizeInCorrection_ = msf_tmp::CorrectionStateLengthForType<
        const StateVar_T<type_T, name_T>&>::value,
    ///The size of this state in the state vector
    sizeInState_ = msf_tmp::StateLengthForType<const StateVar_T<type_T, name_T>&>::value
  };
  typedef Eigen::Matrix<double, sizeInCorrection_, sizeInCorrection_> Q_T;

  Q_T Q;  ///< The noise covariance matrix block of this state.
  value_t state_;  ///< The state variable of this state.
  bool hasResetValue;  //<Indicating that this statevariable has a reset value
                       // to be applied to the state on init.
  StateVar_T() {
    hasResetValue = false;
    Q.setZero();
  }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * \brief The state vector containing all the state variables for this EKF
 * configuration.
 */
template<typename StateSeq_T, typename StateDef_T>
struct GenericState_T {
 public:
  ///The state vector defining the state variables of this EKF.
  typedef StateSeq_T StateSequence_T;
  ///<The enums of the state variables.
  typedef StateDef_T StateDefinition_T;

  friend class msf_core::MSF_Core<
      GenericState_T<StateSequence_T, StateDefinition_T> >;
  friend struct msf_core::copyNonPropagationStates<GenericState_T>;
  friend class msf_core::MSF_InitMeasurement<
      GenericState_T<StateSequence_T, StateDefinition_T> >;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  enum {
    nStateVarsAtCompileTime = boost::fusion::result_of::size < StateSequence_T
        > ::type::value,  ///<N state vars.
    nErrorStatesAtCompileTime = msf_tmp::CountStates<StateSequence_T,
        msf_tmp::CorrectionStateLengthForType>::value,  ///<N error states.
    nStatesAtCompileTime = msf_tmp::CountStates<StateSequence_T,
        msf_tmp::StateLengthForType>::value,  ///<N total states.
    nCoreStatesAtCompileTime = msf_tmp::CountStates<StateSequence_T,
        msf_tmp::CoreStateLengthForType>::value,  ///<N total core states.
    nCoreErrorStatesAtCompileTime = msf_tmp::CountStates<StateSequence_T,
        msf_tmp::CoreErrorStateLengthForType>::value,  ///<N total core error states.
    nPropagatedCoreStatesAtCompileTime = msf_tmp::CountStates<StateSequence_T,
        msf_tmp::PropagatedCoreStateLengthForType>::value,  ///<N total core states with propagation.
    nPropagatedCoreErrorStatesAtCompileTime = msf_tmp::CountStates<
        StateSequence_T, msf_tmp::PropagatedCoreErrorStateLengthForType>::value  ///<N total error states with propagation.
  };

 private:

  /**
   * \brief Returns the stateVar at position INDEX in the state list.
   * Non const version only for msf_core use. You must not make these functions
   * public. Instead const_cast the state object to const to use the overload.
   */
  template<int INDEX>
  inline typename boost::fusion::result_of::at_c<StateSequence_T, INDEX>::type
  getStateVar();

  /**
   * \brief Returns the state at position INDEX in the state list, non const
   * version. You must not make these functions public. Instead const_cast the
   * state object to const to use the overload.
   */
  template<int INDEX>
  inline typename msf_tmp::StripReference<
      typename boost::fusion::result_of::at_c<StateSequence_T, INDEX>::type>::result_t::value_t&
  get();

 public:

  typedef Eigen::Matrix<double, nErrorStatesAtCompileTime,
      nErrorStatesAtCompileTime> P_type;  ///< Type of the error state
                                          // covariance matrix.
  typedef P_type F_type;
  typedef P_type Q_type;

  StateSequence_T statevars;  ///< The actual state variables.

  // system inputs
  Eigen::Matrix<double, 3, 1> w_m;         ///< Angular velocity from IMU.
  Eigen::Matrix<double, 3, 1> a_m;         ///< Linear acceleration from IMU.

  double time; 	///< Time of this state estimate.
  P_type P;  ///< Error state covariance.
  F_type Fd;   ///< Discrete state propagation matrix.
  Q_type Qd;   ///< Discrete propagation noise matrix.

  GenericState_T() {
    time = -1;
    P.setZero();
    Qd.setZero();
    Fd.setIdentity();
    reset();
  }

  /**
   * \brief Apply the correction vector to all state vars.
   */
  inline void correct(
      const Eigen::Matrix<double, nErrorStatesAtCompileTime, 1>& correction);

  /**
   * \brief Returns the Q-block of the state at position INDEX in the state list,
   * not allowed for core states.
   */
  template<int INDEX>
  inline typename msf_tmp::StripReference<
      typename boost::fusion::result_of::at_c<StateSequence_T, INDEX>::type>::result_t::Q_T&
  getQBlock();

  /**
   * \brief Returns the Q-block of the state at position INDEX in the state list,
   * also possible for core states, since const.
   */
  template<int INDEX>
  inline const typename msf_tmp::StripReference<
      typename boost::fusion::result_of::at_c<StateSequence_T, INDEX>::type>::result_t::Q_T&
  getQBlock() const;

  /**
   * \brief Reset the state
   * 3D vectors: 0;
   * quaternion: unit quaternion;
   * scale: 1;
   * time:0;
   * Error covariance: zeros.
   */
  void reset(
      msf_core::StateVisitor<GenericState_T<StateSequence_T, StateDefinition_T> >* usercalc =
          nullptr);

  /**
   * \brief Write the covariance corresponding to position and attitude to cov.
   */
  void getPoseCovariance(
      geometry_msgs::PoseWithCovariance::_covariance_type & cov);

  /**
   * \brief Assembles a PoseWithCovarianceStamped message from the state.
   * \note It does not set the header.
   */
  void toPoseMsg(geometry_msgs::PoseWithCovarianceStamped & pose);

  /**
   * \brief Assemble an ExtState message from the state.
   * \note It does not set the header.
   */
  void toExtStateMsg(sensor_fusion_comm::ExtState & state);

  /***
   * \brief Assemble a DoubleArrayStamped message from the state.
   * \note It does not set the header.
   */
  void toFullStateMsg(sensor_fusion_comm::DoubleArrayStamped & state);

  /**
   * \brief Assembles a DoubleArrayStamped message from the state.
   * \note It does not set the header
   */
  void toCoreStateMsg(sensor_fusion_comm::DoubleArrayStamped & state);

  /**
   * \brief Assembles a DoubleMatrixStamped message from the core error state covariance.
   * \note It does not set the header
   */
  void getCoreCovariance(sensor_fusion_comm::DoubleMatrixStamped & cov);

  /**
   * \brief Assembles a DoubleMatrixStamped message from the aux error state covariance.
   * \note It does not set the header
   */
  void getAuxCovariance(sensor_fusion_comm::DoubleMatrixStamped & cov);

  /**
   * \brief Assembles a DoubleMatrixStamped message from the core-aux error state covariance.
   * \note It does not set the header
   */
  void getCoreAuxCovariance(sensor_fusion_comm::DoubleMatrixStamped & cov);

  /**
   * \brief Returns all values as an eigen vector.
   */
  Eigen::Matrix<double, nCoreStatesAtCompileTime, 1> toEigenVector();

  /**
   * Returns a vector of int pairs with enum index in errorstate and numblocks.
   */
  void calculateIndicesInErrorState(
      std::vector<std::tuple<int, int, int> >& vec);

  /**
   * \brief Returns a string describing the state.
   */
  std::string print();

  /**
   * \brief Returns whether the state is sane. No NaN no inf.
   */
  bool checkStateForNumeric();

  /**
   * \brief Returns the state at position INDEX in the state list, const
   * version.
   */
  template<int INDEX>
  inline const typename msf_tmp::StripReference<
      typename boost::fusion::result_of::at_c<StateSequence_T, INDEX>::type>::result_t::value_t&
  get() const;

  /**
   * \brief Returns the stateVar at position INDEX in the state list,
   * const version.
   */
  template<int INDEX>
  inline typename msf_tmp::AddConstReference<
      typename boost::fusion::result_of::at_c<StateSequence_T, INDEX>::type>::result_t
  getStateVar() const;

  /**
   * \brief Sets state at position INDEX in the state list, fails for core
   * states at compile time.
   */
  template<int INDEX>
  inline void
  set(const typename msf_tmp::StripConstReference<
      typename boost::fusion::result_of::at_c<StateSequence_T, INDEX>::type>::result_t::value_t& newvalue);

  /*
   * Clears the crosscovariance entries of a given state in P
   */
  template<int INDEX>
  inline void
  clearCrossCov();

};

/**
 * \brief Comparator for the state objects. sorts by time asc.
 */
template<typename stateSequence_T, typename stateDefinition_T>
class sortStates {
 public:
  /**
   * \brief Implements the sorting by time.
   */
  bool operator()(
      const GenericState_T<stateSequence_T, stateDefinition_T>& lhs,
      const GenericState_T<stateSequence_T, stateDefinition_T>&rhs) const {
    return (lhs.time_ < rhs.time_);
  }
};

}

#include <msf_core/implementation/msf_state_.hpp>

#endif /* MSF_STATE_H_ */
