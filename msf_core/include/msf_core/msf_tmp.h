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

/*
 * DO NOT MODIFY THIS FILE UNLESS YOU REALLY KNOW WHAT YOU ARE DOING
 * IF THERE ARE COMPILER ERRORS or WARNINGS IN THIS FILE IT IS PROBABLY
 * BECAUSE YOU WHERE DOING SOMETHING WRONG WHEN USING THE CODE
 * AND NOT BECAUSE THERE ARE ERRORS HERE.
 *
 */

#ifndef MSF_TMP_H_
#define MSF_TMP_H_

#include <msf_core/msf_fwds.h>
#include <msf_core/msf_typetraits.h>
#include <msf_core/eigen_utils.h>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <sstream>
#include <iostream>
#include <iomanip>

#ifdef FUSION_MAX_VECTOR_SIZE
#undef FUSION_MAX_VECTOR_SIZE
#endif
#define FUSION_MAX_VECTOR_SIZE 15  // Maximum number of statevariables
// (can be set to a larger value).
#include <boost/lexical_cast.hpp>
#include <boost/preprocessor/punctuation/comma.hpp>
#include <boost/fusion/include/vector.hpp>
#include <boost/fusion/include/size.hpp>
#include <boost/fusion/include/at_c.hpp>
#include <boost/fusion/include/at.hpp>
#include <boost/fusion/include/for_each.hpp>
#include <boost/fusion/include/equal_to.hpp>
#include <boost/fusion/include/begin.hpp>
#include <boost/fusion/include/end.hpp>
#include <boost/fusion/include/next.hpp>
#include <boost/mpl/bool.hpp>
#include <boost/fusion/include/at_key.hpp>

// This namespace contains the msf metaprogramming tools.
namespace msf_tmp {

/**
 * \brief Output a compile time constant by overflowing a char, producing a
 * warning.
 */
template<size_t size>
struct overflow {
  operator char() {
    return size + 9999;
  }
};
// Always overflow, also for negative values.
/**
 * \brief Output a compile time constant.
 */
template<int VALUE>
void EchoCompileTimeConstant() {
  char(overflow<VALUE>());
  return;
}

/**
 * \brief Runtime output of stateVariable types.
 */
template<typename T> struct EchoStateVarType;

/**
 * \brief Runtime output of stateVariable types for const ref eigen matrices.
 */
template<int NAME, int N, int STATE_T, int OPTIONS>
struct EchoStateVarType<
    const msf_core::StateVar_T<Eigen::Matrix<double, N, 1>, NAME, STATE_T,
        OPTIONS>&> {
  static std::string Value() {
    return "const ref Eigen::Matrix<double, " + boost::lexical_cast
        < std::string > (N) + ", 1>";
  }
};
/**
 * \brief Runtime output of stateVariable types for const ref eigen quaternions.
 */
template<int NAME, int STATE_T, int OPTIONS>
struct EchoStateVarType<
    const msf_core::StateVar_T<Eigen::Quaterniond, NAME, STATE_T, OPTIONS>&> {
  static std::string Value() {
    return "const ref Eigen::Quaterniond";
  }
};
/**
 * \brief Runtime output of stateVariable types for eigen matrices.
 */
template<int NAME, int N, int STATE_T, int OPTIONS>
struct EchoStateVarType<
    msf_core::StateVar_T<Eigen::Matrix<double, N, 1>, NAME, STATE_T, OPTIONS> > {
  static std::string Value() {
    return "Eigen::Matrix<double, " + boost::lexical_cast < std::string
        > (N) + ", 1>";
  }
};
/**
 * \brief Runtime output of stateVariable types for eigen matrices.
 */
template<int NAME, int STATE_T, int OPTIONS>
struct EchoStateVarType<
    msf_core::StateVar_T<Eigen::Quaterniond, NAME, STATE_T, OPTIONS> > {
  static std::string Value() {
    return "Eigen::Quaterniond";
  }
};

namespace {  // For internal use only:
// The number of entries in the correction vector for a given state var.
template<typename T>
struct CorrectionStateLengthForType;
template<int NAME, int N, int STATE_T, int OPTIONS>
struct CorrectionStateLengthForType<
    const msf_core::StateVar_T<Eigen::Matrix<double, N, 1>, NAME, STATE_T,
        OPTIONS>&> {
  enum {
    value = N
  };
};
template<int NAME, int STATE_T, int OPTIONS>
struct CorrectionStateLengthForType<
    const msf_core::StateVar_T<Eigen::Quaterniond, NAME, STATE_T, OPTIONS>&> {
  enum {
    value = 3
  };
};
template<>
struct CorrectionStateLengthForType<const mpl_::void_&> {
  enum {
    value = 0
  };
};

// The number of entries in the state for a given state var.
template<typename T>
struct StateLengthForType;
template<int NAME, int N, int STATE_T, int OPTIONS>
struct StateLengthForType<
    const msf_core::StateVar_T<Eigen::Matrix<double, N, 1>, NAME, STATE_T,
        OPTIONS>&> {
  enum {
    value = N
  };
};
template<int NAME, int STATE_T, int OPTIONS>
struct StateLengthForType<
    const msf_core::StateVar_T<Eigen::Quaterniond, NAME, STATE_T, OPTIONS>&> {
  enum {
    value = 4
  };
};
template<>
struct StateLengthForType<const mpl_::void_&> {
  enum {
    value = 0
  };
};

// The number of entries in the state for a given state var if it is core state.
template<typename T>
struct CoreStateLengthForType;
template<int NAME, int N, int STATE_T, int OPTIONS>
struct CoreStateLengthForType<
    const msf_core::StateVar_T<Eigen::Matrix<double, N, 1>, NAME, STATE_T,
        OPTIONS>&> {
  enum {
    value = 0
  };
  // Not a core state, so length is zero.
};
template<int NAME, int STATE_T, int OPTIONS>
struct CoreStateLengthForType<
    const msf_core::StateVar_T<Eigen::Quaterniond, NAME, STATE_T, OPTIONS>&> {
  enum {
    value = 0
  };
  // Not a core state, so length is zero.
};
template<int NAME, int OPTIONS, int N>
struct CoreStateLengthForType<
    const msf_core::StateVar_T<Eigen::Matrix<double, N, 1>, NAME,
        msf_core::CoreStateWithoutPropagation, OPTIONS>&> {
  enum {
    value = N
  };
};
template<int NAME, int OPTIONS>
struct CoreStateLengthForType<
    const msf_core::StateVar_T<Eigen::Quaterniond, NAME,
        msf_core::CoreStateWithoutPropagation, OPTIONS>&> {
  enum {
    value = 4
  };
};
template<int NAME, int OPTIONS, int N>
struct CoreStateLengthForType<
    const msf_core::StateVar_T<Eigen::Matrix<double, N, 1>, NAME,
        msf_core::CoreStateWithPropagation, OPTIONS>&> {
  enum {
    value = N
  };
};
template<int NAME, int OPTIONS>
struct CoreStateLengthForType<
    const msf_core::StateVar_T<Eigen::Quaterniond, NAME,
        msf_core::CoreStateWithPropagation, OPTIONS>&> {
  enum {
    value = 4
  };
};
template<>
struct CoreStateLengthForType<const mpl_::void_&> {
  enum {
    value = 0
  };
};

// The number of entries in the error state for a given state var if it is core state.
template<typename T>
struct CoreErrorStateLengthForType;
template<int NAME, int N, int STATE_T, int OPTIONS>
struct CoreErrorStateLengthForType<
    const msf_core::StateVar_T<Eigen::Matrix<double, N, 1>, NAME, STATE_T,
        OPTIONS>&> {
  enum {
    value = 0
  };
  // Not a core state, so length is zero.
};
template<int NAME, int STATE_T, int OPTIONS>
struct CoreErrorStateLengthForType<
    const msf_core::StateVar_T<Eigen::Quaterniond, NAME, STATE_T, OPTIONS>&> {
  enum {
    value = 0
  };
  // Not a core state, so length is zero.
};
template<int NAME, int OPTIONS, int N>
struct CoreErrorStateLengthForType<
    const msf_core::StateVar_T<Eigen::Matrix<double, N, 1>, NAME,
        msf_core::CoreStateWithoutPropagation, OPTIONS>&> {
  enum {
    value = N
  };
};
template<int NAME, int OPTIONS>
struct CoreErrorStateLengthForType<
    const msf_core::StateVar_T<Eigen::Quaterniond, NAME,
        msf_core::CoreStateWithoutPropagation, OPTIONS>&> {
  enum {
    value = 3
  };
};
template<int NAME, int OPTIONS, int N>
struct CoreErrorStateLengthForType<
    const msf_core::StateVar_T<Eigen::Matrix<double, N, 1>, NAME,
        msf_core::CoreStateWithPropagation, OPTIONS>&> {
  enum {
    value = N
  };
};
template<int NAME, int OPTIONS>
struct CoreErrorStateLengthForType<
    const msf_core::StateVar_T<Eigen::Quaterniond, NAME,
        msf_core::CoreStateWithPropagation, OPTIONS>&> {
  enum {
    value = 3
  };
};
template<>
struct CoreErrorStateLengthForType<const mpl_::void_&> {
  enum {
    value = 0
  };
};

// The number of entries in the state for a given state var if it is core state
// with propagation.
template<typename T>
struct PropagatedCoreStateLengthForType;
template<int NAME, int N, int STATE_T, int OPTIONS>
struct PropagatedCoreStateLengthForType<
    const msf_core::StateVar_T<Eigen::Matrix<double, N, 1>, NAME, STATE_T,
        OPTIONS>&> {
  enum {
    value = 0
  };
  // Not a core state, so length is zero.
};
template<int NAME, int STATE_T, int OPTIONS>
struct PropagatedCoreStateLengthForType<
    const msf_core::StateVar_T<Eigen::Quaterniond, NAME, STATE_T, OPTIONS>&> {
  enum {
    value = 0
  };
  // Not a core state, so length is zero.
};
template<int NAME, int OPTIONS, int N>
struct PropagatedCoreStateLengthForType<
    const msf_core::StateVar_T<Eigen::Matrix<double, N, 1>, NAME,
        msf_core::CoreStateWithPropagation, OPTIONS>&> {
  enum {
    value = N
  };
};
template<int NAME, int OPTIONS>
struct PropagatedCoreStateLengthForType<
    const msf_core::StateVar_T<Eigen::Quaterniond, NAME,
        msf_core::CoreStateWithPropagation, OPTIONS>&> {
  enum {
    value = 4
  };
};
template<>
struct PropagatedCoreStateLengthForType<const mpl_::void_&> {
  enum {
    value = 0
  };
};

// The number of entries in the error state for a given state var if it is core
// state with propagation.
template<typename T>
struct PropagatedCoreErrorStateLengthForType;
template<int NAME, int N, int STATE_T, int OPTIONS>
struct PropagatedCoreErrorStateLengthForType<
    const msf_core::StateVar_T<Eigen::Matrix<double, N, 1>, NAME, STATE_T,
        OPTIONS>&> {
  enum {
    value = 0
  };
  // Not a core state, so length is zero.
};
template<int NAME, int STATE_T, int OPTIONS>
struct PropagatedCoreErrorStateLengthForType<
    const msf_core::StateVar_T<Eigen::Quaterniond, NAME, STATE_T, OPTIONS>&> {
  enum {
    value = 0
  };
  // Not a core state, so length is zero.
};
template<int NAME, int OPTIONS, int N>
struct PropagatedCoreErrorStateLengthForType<
    const msf_core::StateVar_T<Eigen::Matrix<double, N, 1>, NAME,
        msf_core::CoreStateWithPropagation, OPTIONS>&> {
  enum {
    value = N
  };
};
template<int NAME, int OPTIONS>
struct PropagatedCoreErrorStateLengthForType<
    const msf_core::StateVar_T<Eigen::Quaterniond, NAME,
        msf_core::CoreStateWithPropagation, OPTIONS>&> {
  enum {
    value = 3
  };
};
template<>
struct PropagatedCoreErrorStateLengthForType<const mpl_::void_&> {
  enum {
    value = 0
  };
};

// Return the number a state has in the enum.
template<typename T>
struct GetEnumStateName;
template<typename U, int NAME, int STATE_T, int OPTIONS>
struct GetEnumStateName<const msf_core::StateVar_T<U, NAME, STATE_T, OPTIONS>&> {
  enum {
    value = NAME
  };
};
template<typename U, int NAME, int STATE_T, int OPTIONS>
struct GetEnumStateName<msf_core::StateVar_T<U, NAME, STATE_T, OPTIONS> > {
  enum {
    value = NAME
  };
};
template<> struct GetEnumStateName<const mpl_::void_&> {
  enum {
    value = -1
  };
  // Must not change this.
};
template<> struct GetEnumStateName<mpl_::void_> {
  enum {
    value = -1
  };
  // Must not change this.
};

template<typename T>
struct IsQuaternionType;
template<int NAME, int STATE_T, int OPTIONS, int M, int N>
struct IsQuaternionType<
    const msf_core::StateVar_T<Eigen::Matrix<double, M, N>, NAME, STATE_T,
        OPTIONS>&> {
  enum {
    value = false
  };
};
template<int NAME, int STATE_T, int OPTIONS, int M, int N>
struct IsQuaternionType<
    msf_core::StateVar_T<Eigen::Matrix<double, M, N>, NAME, STATE_T, OPTIONS> > {
  enum {
    value = false
  };
};
template<int NAME, int STATE_T, int OPTIONS>
struct IsQuaternionType<
    const msf_core::StateVar_T<Eigen::Quaterniond, NAME, STATE_T, OPTIONS>&> {
  enum {
    value = true
  };
};
template<int NAME, int STATE_T, int OPTIONS>
struct IsQuaternionType<
    msf_core::StateVar_T<Eigen::Quaterniond, NAME, STATE_T, OPTIONS> > {
  enum {
    value = true
  };
};
template<>
struct IsQuaternionType<const mpl_::void_&> {
  enum {
    value = false
  };
};
template<>
struct IsQuaternionType<mpl_::void_> {
  enum {
    value = false
  };
};

/**
 * \brief Return whether a state is nontemporaldrifting.
 */
template<typename T>
struct GetStateIsNonTemporalDrifting;
template<typename U, int NAME, int STATE_T, int OPTIONS>
struct GetStateIsNonTemporalDrifting<
    const msf_core::StateVar_T<U, NAME, STATE_T, OPTIONS>&> {
  enum {
    value = static_cast<int>(msf_core::StateVar_T<U, NAME, STATE_T, OPTIONS>::statetype_)
        == static_cast<int>(msf_core::AuxiliaryNonTemporalDrifting)
  };
};
template<typename U, int NAME, int STATE_T, int OPTIONS>
struct GetStateIsNonTemporalDrifting<
    msf_core::StateVar_T<U, NAME, STATE_T, OPTIONS> > {
  enum {
    value = static_cast<int>(msf_core::StateVar_T<U, NAME, STATE_T, OPTIONS>::statetype_)
        == static_cast<int>(msf_core::AuxiliaryNonTemporalDrifting)
  };
};
template<> struct GetStateIsNonTemporalDrifting<const mpl_::void_&> {
  enum {
    value = false
  };
};
template<> struct GetStateIsNonTemporalDrifting<mpl_::void_> {
  enum {
    value = false
  };
};

/**
 * \brief Count some entity in the state vector using the counter class
 * provided.
 */
template<typename Sequence, template<typename > class Counter, typename First,
    typename Last, bool T>
struct CountStatesLinear;
template<typename Sequence, template<typename > class Counter, typename First,
    typename Last>
struct CountStatesLinear<Sequence, Counter, First, Last, true> {
  enum {  // The end does not add entries.
    value = 0
  };
};
template<typename Sequence, template<typename > class Counter, typename First,
    typename Last>
struct CountStatesLinear<Sequence, Counter, First, Last, false> {
  typedef typename boost::fusion::result_of::next<First>::type Next;
  typedef typename boost::fusion::result_of::deref<First>::type current_Type;
  enum {  // The length of the current state plus the tail of the list.
    value = Counter<current_Type>::value
        + CountStatesLinear<Sequence, Counter, Next, Last,
            SameType<typename msf_tmp::StripConstReference<First>::result_t,
                typename msf_tmp::StripConstReference<Last>::result_t>::value>::value
  };
};

/**
 * \brief Helpers for CheckCorrectIndexing call{
 */
template<typename Sequence, typename First, typename Last, int CurrentIdx,
    bool T>
struct CheckStateIndexing;
template<typename Sequence, typename First, typename Last, int CurrentIdx>
struct CheckStateIndexing<Sequence, First, Last, CurrentIdx, true> {
  enum {  //no index no name, no indexing errors
    indexingerrors = 0
  };
};
template<typename Sequence, typename First, typename Last, int CurrentIdx>
struct CheckStateIndexing<Sequence, First, Last, CurrentIdx, false> {
  typedef typename boost::fusion::result_of::next<First>::type Next;
  typedef typename boost::fusion::result_of::deref<First>::type current_Type;
  enum {  // The length of the current state plus the tail of the list.
    // Only evaluate if not at the end of the list.
    idxInEnum = boost::mpl::if_c<GetEnumStateName<current_Type>::value != -1,
    // If we're not at the end of the list, use the calculated enum index.
        boost::mpl::int_<GetEnumStateName<current_Type>::value>,
        //Else use the current index, so the assertion doesn't fail.
        boost::mpl::int_<CurrentIdx> >::type::value,

    idxInState = CurrentIdx,

    indexingerrors = boost::mpl::if_c<idxInEnum == idxInState,
    // Error here.
        boost::mpl::int_<0>, boost::mpl::int_<1> >::type::value +
    // And errors of other states.
        CheckStateIndexing<Sequence, Next, Last, CurrentIdx + 1,
            SameType<typename msf_tmp::StripConstReference<First>::result_t,
                typename msf_tmp::StripConstReference<Last>::result_t>::value>::indexingerrors

  };
 private:
  // Error the ordering in the enum defining the names of the states is not the
  // same ordering as in the type vector for the states.
  static_assert(indexingerrors==0, "Error the ordering in the enum defining the names _ "
      "of the states is not the same ordering as in the type vector for the states");
};
//}

/**
 * \brief Return the state type of a given enum value.
 */
template<typename TypeList, int INDEX>
struct GetEnumStateType {
  typedef typename boost::fusion::result_of::at_c<TypeList, INDEX>::type value;
};

/**
 * \brief Return void type if index is -1.
 */
template<typename TypeList>
struct GetEnumStateType<TypeList, -1> {
  typedef mpl_::void_ value;
};

/**
 * \brief Helper for GetStartIndex{
 */
template<typename Sequence, typename StateVarT,
    template<typename > class OffsetCalculator, typename First, typename Last,
    bool TypeFound, int CurrentVal, bool EndOfList>
struct ComputeStartIndex;
template<typename Sequence, typename StateVarT,
    template<typename > class OffsetCalculator, typename First, typename Last,
    int CurrentVal>
struct ComputeStartIndex<Sequence, StateVarT, OffsetCalculator, First, Last,
    false, CurrentVal, true> {
  enum { 		// Return error code if end of list reached and type not found.
    value = -1
  };
};
template<typename Sequence, typename StateVarT,
    template<typename > class OffsetCalculator, typename First, typename Last,
    bool TypeFound, int CurrentVal>
struct ComputeStartIndex<Sequence, StateVarT, OffsetCalculator, First, Last,
    TypeFound, CurrentVal, false> {
  enum { 		// We found the type, do not add additional offset.
    value = 0
  };
};
template<typename Sequence, typename StateVarT,
    template<typename > class OffsetCalculator, typename First, typename Last,
    int CurrentVal>
struct ComputeStartIndex<Sequence, StateVarT, OffsetCalculator, First, Last,
    false, CurrentVal, false> {
  typedef typename boost::fusion::result_of::next<First>::type Next;
  typedef typename boost::fusion::result_of::deref<First>::type currentType;

  enum { 		// The length of the current state plus the tail of the list.
    value = CurrentVal
        + ComputeStartIndex<Sequence, StateVarT, OffsetCalculator, Next, Last,
            SameType<
                typename msf_tmp::StripConstReference<currentType>::result_t,
                typename msf_tmp::StripConstReference<StateVarT>::result_t>::value,
            OffsetCalculator<currentType>::value,
            SameType<typename msf_tmp::StripConstReference<First>::result_t,
                typename msf_tmp::StripConstReference<Last>::result_t>::value>::value
  };
};
//}

/**
 * \brief Helper for find best nontemporal drifting state{
 */
template<typename Sequence, typename First, typename Last, int CurrentBestIdx,
    bool foundQuaternion, bool EndOfList>
struct FindBestNonTemporalDriftingStateImpl;
template<typename Sequence, typename First, typename Last, int CurrentBestIdx,
    bool foundQuaternion>
struct FindBestNonTemporalDriftingStateImpl<Sequence, First, Last,
    CurrentBestIdx, foundQuaternion, true> {
  enum { 		// Set to the current index found, which was initially set to -1.
    value = CurrentBestIdx
  };
};
template<typename Sequence, typename First, typename Last, int CurrentBestIdx,
    bool foundQuaternion>
struct FindBestNonTemporalDriftingStateImpl<Sequence, First, Last,
    CurrentBestIdx, foundQuaternion, false> {
  typedef typename boost::fusion::result_of::next<First>::type Next;
  typedef typename boost::fusion::result_of::deref<First>::type current_Type;
  enum {
    idxInState = GetEnumStateName<
    // We assert before calling that the indexing is correct.
        typename msf_tmp::StripConstReference<current_Type>::result_t>::value,
    IsCurrentStateAQuaternion = IsQuaternionType<
        typename msf_tmp::StripConstReference<current_Type>::result_t>::value,
    IsCurrentStateNonTemporalDrifting = GetStateIsNonTemporalDrifting<
        typename msf_tmp::StripConstReference<current_Type>::result_t>::value,
    isQuaternionAndNonTemporalDrifting = IsCurrentStateAQuaternion == true
        && IsCurrentStateNonTemporalDrifting == true,
    isNonTemporalDriftingPositionAndNoQuaternionHasBeenFound = IsCurrentStateNonTemporalDrifting
        == true && foundQuaternion == false,

    currBestIdx = boost::mpl::if_c<isQuaternionAndNonTemporalDrifting,
        //Set to this index if quaternion and nontemporal drifting
        boost::mpl::int_<idxInState>,
        boost::mpl::int_<
            boost::mpl::if_c<
                isNonTemporalDriftingPositionAndNoQuaternionHasBeenFound,
                // If we havent found a quaternion yet and this state is non
                // temporal drifting, we take this one.
                boost::mpl::int_<idxInState>,
                // Else we take the current best one.
                boost::mpl::int_<CurrentBestIdx> >::type::value> >::type::value,

    value = FindBestNonTemporalDriftingStateImpl<Sequence, Next, Last,
        currBestIdx, foundQuaternion || isQuaternionAndNonTemporalDrifting,
        SameType<typename msf_tmp::StripConstReference<First>::result_t,
            typename msf_tmp::StripConstReference<Last>::result_t>::value>::value

  };
};

}  // anonymous namespace

/**
 * \brief Checks whether the ordering in the vector is the same as in the enum
 * this ordering is something that strictly must not change.
 */
template<typename Sequence>
struct CheckCorrectIndexing {
  typedef typename boost::fusion::result_of::begin<Sequence const>::type First;
  typedef typename boost::fusion::result_of::end<Sequence const>::type Last;
 private:
  enum {
    startindex = 0,  // Must not change this.
  };
 public:
  enum {
    indexingerrors = CheckStateIndexing<Sequence, First, Last, startindex,
        SameType<typename msf_tmp::StripConstReference<First>::result_t,
            typename msf_tmp::StripConstReference<Last>::result_t>::value>::indexingerrors
  };
};

/**
 * \brief Find the best nontemporal drifting state in the full state vector at
 * compile time.
 *
 * The goal of this tmp code is to find the best nontemporal drifting state in the
 * full state vector at compile time. We therefore search the state variable list
 * for states that the user has marked as non temporal drifting. If furthermore
 * prefer quaternions over euclidean states. This function will return -1 if no
 * suitable state has been found.
 */
template<typename Sequence>
struct IndexOfBestNonTemporalDriftingState {
  typedef typename boost::fusion::result_of::begin<Sequence const>::type First;
  typedef typename boost::fusion::result_of::end<Sequence const>::type Last;
 private:
  enum {
    bestindex = -1,  // Must not change this.
  };
  static_assert(CheckCorrectIndexing<Sequence>::indexingerrors == 0, "The "
      "indexing of the state vector is not the same as in the enum,"
      " but this must be the same");
 public:
  enum {
    value = FindBestNonTemporalDriftingStateImpl<Sequence, First, Last,
        bestindex, false,
        SameType<typename msf_tmp::StripConstReference<First>::result_t,
            typename msf_tmp::StripConstReference<Last>::result_t>::value>::value
  };
};

/**
 * \brief Returns the number of doubles in the state/correction state
 * depending on the counter type supplied.
 */
template<typename Sequence, template<typename > class Counter>
struct CountStates {
  typedef typename boost::fusion::result_of::begin<Sequence const>::type First;
  typedef typename boost::fusion::result_of::end<Sequence const>::type Last;
  enum {
    value = CountStatesLinear<Sequence, Counter, First, Last,
        SameType<typename msf_tmp::StripConstReference<First>::result_t,
            typename msf_tmp::StripConstReference<Last>::result_t>::value>::value
    // will be zero, if no indexing errors, otherwise fails compilation.
        + CheckCorrectIndexing<Sequence>::indexingerrors
  };
};

/**
 * \brief Compute start indices in the correction/state vector of a given type.
 */
template<typename Sequence, typename StateVarT,
    template<typename > class Counter>
struct GetStartIndex {
  typedef typename boost::fusion::result_of::begin<Sequence const>::type First;
  typedef typename boost::fusion::result_of::end<Sequence const>::type Last;
  typedef typename boost::fusion::result_of::deref<First>::type currentType;
  enum {
    value = ComputeStartIndex<Sequence, StateVarT, Counter, First, Last,
        SameType<typename msf_tmp::StripConstReference<StateVarT>::result_t,
            typename msf_tmp::StripConstReference<currentType>::result_t>::value,
        0, SameType<First, Last>::value>::value
    // Will be zero, if no indexing errors, otherwise fails compilation.
        + CheckCorrectIndexing<Sequence>::indexingerrors
  };
};

/**
 * \brief Compute start indices in the correction vector of a given type.
 */
template<typename Sequence, int StateEnum>
struct GetStartIndexInCorrection {
  typedef typename boost::fusion::result_of::begin<Sequence const>::type First;
  typedef typename boost::fusion::result_of::end<Sequence const>::type Last;
  typedef typename boost::fusion::result_of::deref<First>::type currentType;
  enum {
    value = msf_tmp::GetStartIndex<Sequence,
        typename msf_tmp::GetEnumStateType<Sequence, StateEnum>::value,
        msf_tmp::CorrectionStateLengthForType>::value
  };
};

/**
 * \brief Reset the EKF state in a boost fusion unrolled call.
 */
struct ResetState {
  template<int NAME, int N, int STATE_T, int OPTIONS>
  void operator()(
      msf_core::StateVar_T<Eigen::Matrix<double, N, 1>, NAME, STATE_T, OPTIONS>& t) const {
    t.state_.setZero();
  }
  template<int NAME, int STATE_T, int OPTIONS>
  void operator()(
      msf_core::StateVar_T<Eigen::Quaterniond, NAME, STATE_T, OPTIONS>& t) const {
    t.state_.setIdentity();
  }
};

/**
 * \brief Copy states from previous to current states, for which there is no
 * propagation in a boost fusion unrolled call.
 */
template<typename stateVarT>
struct CopyInitStates {
  CopyInitStates(const stateVarT& oldstate)
      : oldstate_(oldstate) { }
  template<typename T, int NAME, int STATE_T, int OPTIONS>
  void operator()(msf_core::StateVar_T<T, NAME, STATE_T, OPTIONS>& t) const {
    if (oldstate_.template GetStateVariable<NAME>().hasResetValue) {
      // Copy value from old state to new state var.
        t = oldstate_.template GetStateVariable<NAME>();
      }
  }

  private:
    const stateVarT& oldstate_;
  };

    /**
     * \brief Copy states from previous to current states, for which there is no
     * propagation.
     */
template<typename stateVarT>
struct CopyNonPropagationStates {
  CopyNonPropagationStates(const stateVarT& oldstate)
      : oldstate_(oldstate) {
  }
  template<typename T, int NAME, int OPTIONS>
  void operator()(
      msf_core::StateVar_T<T, NAME, msf_core::CoreStateWithPropagation, OPTIONS>& UNUSEDPARAM(t)) const {
    // Nothing to do for the states, which have propagation.
  }
  template<typename T, int NAME, int STATE_T, int OPTIONS>
  void operator()(msf_core::StateVar_T<T, NAME, STATE_T, OPTIONS>& t) const {

    static_assert(
        (msf_tmp::IsPointerType<typename msf_core::StateVar_T<T,
            NAME, STATE_T> >::value == false),
        "The state variable is of pointer type, but this method assumes we can "
        "copy without dereferencing");

    static_assert(
        STATE_T != msf_core::CoreStateWithPropagation,
        "CopyNonPropagationStates was instantiated for a core state. "
        "This is an error.");
    // Copy value from old state to new state var.
    t = oldstate_.template GetStateVariable<NAME>();
  }

 private:
  const stateVarT& oldstate_;
};

/**
 * \brief Copy the user calculated values in the Q-blocks to the main Q matrix.
 */
template<typename stateList_T>
struct CopyQBlocksFromAuxiliaryStatesToQ {
  enum {
    nErrorStatesAtCompileTime = msf_tmp::CountStates<stateList_T,
        msf_tmp::CorrectionStateLengthForType>::value  // N correction states.
  };
  typedef Eigen::Matrix<double, nErrorStatesAtCompileTime,
      nErrorStatesAtCompileTime> Q_T;
  CopyQBlocksFromAuxiliaryStatesToQ(Q_T& Q)
      : Q_(Q) { }
  template<typename T, int NAME, int STATE_T, int OPTIONS>
  void operator()(msf_core::StateVar_T<T, NAME, STATE_T, OPTIONS>& t) const {
    typedef msf_core::StateVar_T<T, NAME, STATE_T, OPTIONS> var_T;
    enum {
      startIdxInCorrection = msf_tmp::GetStartIndex<stateList_T, var_T,
          msf_tmp::CorrectionStateLengthForType>::value,
      sizeInCorrection = var_T::sizeInCorrection_
    };

    static_assert(
        static_cast<int>(var_T::statetype_) !=
        static_cast<int>(msf_core::CoreStateWithPropagation) &&
        static_cast<int>(var_T::statetype_) !=
        static_cast<int>(msf_core::CoreStateWithoutPropagation),
        "CopyQBlocksFromAuxiliaryStatesToQ was instantiated for a core state. "
        "This is an error.");

    static_assert(
        static_cast<int>(sizeInCorrection) ==
        static_cast<int>(var_T::Q_T::ColsAtCompileTime) &&
        static_cast<int>(sizeInCorrection) ==
        static_cast<int>(var_T::Q_T::RowsAtCompileTime),
        "CopyQBlocksFromAuxiliaryStatesToQ size of Matrix Q stored with the stateVar,"
        "is not the same as the reported dimension in the correction vector");

    Q_.template block<sizeInCorrection, sizeInCorrection>(startIdxInCorrection,
                                                          startIdxInCorrection) =
        t.Q;
  }
  template<typename T, int NAME, int OPTIONS>
  void operator()(
      msf_core::StateVar_T<T, NAME, msf_core::CoreStateWithPropagation, OPTIONS>& UNUSEDPARAM(t)) const {
    // Nothing to do for the states, which have propagation, because Q
    // calculation is done in msf_core.
  }
  template<typename T, int NAME, int OPTIONS>
  void operator()(
      msf_core::StateVar_T<T, NAME, msf_core::CoreStateWithoutPropagation,
          OPTIONS>& UNUSEDPARAM(t)) const {
    // Nothing to do for the states, which have propagation, because Q
    // calculation is done in msf_core.
  }
 private:
  Q_T& Q_;
};

/**
 * \brief Apply EKF corrections depending on the stateVar type.
 */
template<typename T, typename stateList_T>
struct CorrectState {
  CorrectState(T& correction)
      : data_(correction) {
  }
  template<int NAME, int N, int STATE_T, int OPTIONS>
  void operator()(
      msf_core::StateVar_T<Eigen::Matrix<double, N, 1>, NAME, STATE_T, OPTIONS>& t) const {
    typedef msf_core::StateVar_T<Eigen::Matrix<double, N, 1>, NAME, STATE_T,
        OPTIONS> var_T;
    enum {
      startIdxInCorrection = msf_tmp::GetStartIndex<stateList_T, var_T,
          msf_tmp::CorrectionStateLengthForType>::value
    };
    if (OPTIONS & msf_core::correctionMultiplicative) {
      t.state_ = t.state_.cwiseProduct(
          data_.template block<var_T::sizeInCorrection_, 1>(
              startIdxInCorrection, 0));
    } else {
      t.state_ = t.state_
          + data_.template block<var_T::sizeInCorrection_, 1>(
              startIdxInCorrection, 0);
    }
  }
  template<int NAME, int STATE_T, int OPTIONS>
  void operator()(
      msf_core::StateVar_T<Eigen::Quaterniond, NAME, STATE_T, OPTIONS>& t) const {
    typedef msf_core::StateVar_T<Eigen::Quaterniond, NAME, STATE_T> var_T;
    enum {
      startIdxInCorrection = msf_tmp::GetStartIndex<stateList_T, var_T,
          msf_tmp::CorrectionStateLengthForType>::value
    };
    // Make sure the user does not define bogus options.
    static_assert(
        (OPTIONS & msf_core::correctionMultiplicative) == false,
        "You defined the Quaternion correction to be multiplicative, but this "
        "is anyway done and not an option");

    Eigen::Quaternion<double> qbuff_q = QuaternionFromSmallAngle(
        data_.template block<var_T::sizeInCorrection_, 1>(startIdxInCorrection,
                                                          0));
    t.state_ = t.state_ * qbuff_q;
    t.state_.normalize();
  }
 private:
  T& data_;
};

/**
 * \brief Copies the values of the single state vars to the double array provided.
 */
template<typename T, typename stateList_T>
struct CoreStatetoDoubleArray {
  CoreStatetoDoubleArray(T& statearray)
      : data_(statearray) {
  }
  template<int NAME, int N, int STATE_T, int OPTIONS>
  void operator()(
      msf_core::StateVar_T<Eigen::Matrix<double, N, 1>, NAME, STATE_T, OPTIONS>& t) const {
    typedef msf_core::StateVar_T<Eigen::Matrix<double, N, 1>, NAME, STATE_T,
        OPTIONS> var_T;
    enum {
      startIdxInState = msf_tmp::GetStartIndex<stateList_T, var_T,
      // Index of the data in the state vector.
          msf_tmp::StateLengthForType>::value
    };
    static_assert(
        static_cast<int>(var_T::statetype_) ==
        static_cast<int>(msf_core::CoreStateWithPropagation) ||
        static_cast<int>(var_T::statetype_) ==
        static_cast<int>(msf_core::CoreStateWithoutPropagation),
        "CoreStatetoDoubleArray: A new statetype was defined, but I don't know "
        "whether this should be written to the double array for core states");
    for (int i = 0; i < var_T::sizeInState_; ++i) {
      data_[startIdxInState + i] = t.state_[i];
    }
  }
  template<int NAME, int STATE_T, int OPTIONS>
  void operator()(
      msf_core::StateVar_T<Eigen::Quaterniond, NAME, STATE_T, OPTIONS>& t) const {
    typedef msf_core::StateVar_T<Eigen::Quaterniond, NAME, STATE_T, OPTIONS> var_T;
    enum {
      startIdxInState = msf_tmp::GetStartIndex<stateList_T, var_T,
      // Index of the data in the state vector.
          msf_tmp::StateLengthForType>::value
    };
    static_assert(
        static_cast<int>(var_T::statetype_) ==
        static_cast<int>(msf_core::CoreStateWithPropagation) ||
        static_cast<int>(var_T::statetype_) ==
        static_cast<int>(msf_core::CoreStateWithoutPropagation),
        "CoreStatetoDoubleArray: A new statetype was defined, but I don't know "
        "whether this should be written to the double array for core states");
    // Copy quaternion values.
    data_[startIdxInState + 0] = t.state_.w();
    data_[startIdxInState + 1] = t.state_.x();
    data_[startIdxInState + 2] = t.state_.y();
    data_[startIdxInState + 3] = t.state_.z();
  }
  template<int NAME, int OPTIONS, int N>
  void operator()(
      msf_core::StateVar_T<Eigen::Matrix<double, N, 1>, NAME,
          msf_core::Auxiliary, OPTIONS>& UNUSEDPARAM(t)) const {
    // Don't copy aux states.
  }
  template<int NAME, int OPTIONS>
  void operator()(
      msf_core::StateVar_T<Eigen::Quaterniond, NAME, msf_core::Auxiliary,
          OPTIONS>& UNUSEDPARAM(t)) const {
    // Don't copy aux states.
  }

  template<int NAME, int OPTIONS, int N>
  void operator()(
      msf_core::StateVar_T<Eigen::Matrix<double, N, 1>, NAME,
          msf_core::AuxiliaryNonTemporalDrifting, OPTIONS>& UNUSEDPARAM(t)) const {
    // Don't copy aux states.
  }
  template<int NAME, int OPTIONS>
  void operator()(
      msf_core::StateVar_T<Eigen::Quaterniond, NAME,
          msf_core::AuxiliaryNonTemporalDrifting, OPTIONS>& UNUSEDPARAM(t)) const {
    // Don't copy aux states.
  }
 private:
  T& data_;
};

/**
 * \brief Copies pairs of statename and index in correction vector to STL
 * container of pairs or tuples.
 */
template<typename T, typename stateList_T>
struct GetIndicesInErrorState {
  GetIndicesInErrorState(T& val)
      : data_(val) {
  }
  template<typename VALUE_T, int NAME, int STATE_T, int OPTIONS>
  void operator()(
      msf_core::StateVar_T<VALUE_T, NAME, STATE_T, OPTIONS>& UNUSEDPARAM(t)) const {
    typedef msf_core::StateVar_T<VALUE_T, NAME, STATE_T, OPTIONS> var_T;
    enum {
      startIdxInState = msf_tmp::GetStartIndex<stateList_T, var_T,
      // Index of the data in the correction vector.
          msf_tmp::CorrectionStateLengthForType>::value,
      lengthInState = var_T::sizeInCorrection_
    };
    data_.push_back(
        typename T::value_type(NAME, startIdxInState, lengthInState));
  }
 private:
  T& data_;
};

/**
 * \brief Return the index of a given state variable number in the state
 * matrices.
 */
template<typename stateVector_T, int INDEX>
struct GetStateIndexInState {
  typedef typename msf_tmp::GetEnumStateType<stateVector_T, INDEX>::value state_type;
  enum {
    value = msf_tmp::GetStartIndex<stateVector_T, state_type,
        msf_tmp::StateLengthForType>::value,
  };
};

/**
 * \brief Return the index of a given state variable number in the error state
 * matrices.
 */
template<typename stateVector_T, int INDEX>
struct GetStateIndexInErrorState {
  typedef typename msf_tmp::GetEnumStateType<stateVector_T, INDEX>::value state_type;
  enum {
    value = msf_tmp::GetStartIndex<stateVector_T, state_type,
        msf_tmp::CorrectionStateLengthForType>::value,
  };
};

/**
 * \brief Copies the values of the single state vars to the string provided.
 */
template<typename STREAM, typename stateList_T>
struct FullStatetoString {
  FullStatetoString(STREAM& data)
      : data_(data) {
  }
  template<int NAME, int N, int STATE_T, int OPTIONS>
  void operator()(
      msf_core::StateVar_T<Eigen::Matrix<double, N, 1>, NAME, STATE_T, OPTIONS>& t) const {
    typedef msf_core::StateVar_T<Eigen::Matrix<double, N, 1>, NAME, STATE_T,
        OPTIONS> var_T;
    enum {
      startIdxInState = msf_tmp::GetStartIndex<stateList_T, var_T,
      // Index of the data in the state vector.
          msf_tmp::StateLengthForType>::value
    };
    data_ << NAME << " : [" << startIdxInState << "-" << startIdxInState + N - 1
        << "]\t : Matrix<" << N << ", 1>         : [" << t.state_.transpose()
        << "]" << std::endl;
  }
  template<int NAME, int STATE_T, int OPTIONS>
  void operator()(
      msf_core::StateVar_T<Eigen::Quaterniond, NAME, STATE_T, OPTIONS>& t) const {
    typedef msf_core::StateVar_T<Eigen::Quaterniond, NAME, STATE_T, OPTIONS> var_T;
    enum {
      startIdxInState = msf_tmp::GetStartIndex<stateList_T, var_T,
      // Index of the data in the state vector.
          msf_tmp::StateLengthForType>::value
    };
    data_ << NAME << " : [" << startIdxInState << "-" << startIdxInState + 3
        << "]\t : Quaternion (w,x,y,z) : " << STREAMQUAT(t.state_) << std::endl;
  }
 private:
  STREAM& data_;
};

/**
 * \brief Copies the values of the single state vars to the double array
 * provided.
 */
template<typename T, typename stateList_T>
struct FullStatetoDoubleArray {
  FullStatetoDoubleArray(T& statearray)
      : data_(statearray) {
  }
  template<int NAME, int N, int STATE_T, int OPTIONS>
  void operator()(
      msf_core::StateVar_T<Eigen::Matrix<double, N, 1>, NAME, STATE_T, OPTIONS>& t) const {
    typedef msf_core::StateVar_T<Eigen::Matrix<double, N, 1>, NAME, STATE_T,
        OPTIONS> var_T;
    enum {
      startIdxInState = msf_tmp::GetStartIndex<stateList_T, var_T,
      // Index of the data in the state vector.
          msf_tmp::StateLengthForType>::value
    };
    for (int i = 0; i < var_T::sizeInState_; ++i) {
      data_[startIdxInState + i] = t.state_[i];
    }
  }
  template<int NAME, int STATE_T, int OPTIONS>
  void operator()(
      msf_core::StateVar_T<Eigen::Quaterniond, NAME, STATE_T, OPTIONS>& t) const {
    typedef msf_core::StateVar_T<Eigen::Quaterniond, NAME, STATE_T, OPTIONS> var_T;
    enum {
      startIdxInState = msf_tmp::GetStartIndex<stateList_T, var_T,
      // Index of the data in the state vector.
          msf_tmp::StateLengthForType>::value
    };
    // Copy quaternion values.
    data_[startIdxInState + 0] = t.state_.w();
    data_[startIdxInState + 1] = t.state_.x();
    data_[startIdxInState + 2] = t.state_.y();
    data_[startIdxInState + 3] = t.state_.z();
  }
 private:
  T& data_;
};
}

#endif  // MSF_TMP_H_
