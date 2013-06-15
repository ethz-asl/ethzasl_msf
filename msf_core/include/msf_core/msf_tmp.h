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

/*
 * DO NOT MODIFY THIS FILE UNLESS YOU REALLY KNOW WHAT YOU ARE DOING
 * IF THERE ARE COMPILER WARNINGS IN THIS FILE IT IS PROBABLY
 * BECAUSE YOU WHERE DOING SOMETHING WRONG WHEN USING THE CODE
 * AND NOT BECAUSE THERE ARE ERRORS HERE.
 *
 */

#ifndef MSF_TMP_HPP_
#define MSF_TMP_HPP_

#include <msf_core/msf_fwds.h>
#include <msf_core/msf_typetraits.tpp>
#include <msf_core/eigen_utils.h>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <sstream>
#include <iostream>
#include <iomanip>

#ifdef FUSION_MAX_VECTOR_SIZE
#undef FUSION_MAX_VECTOR_SIZE
#endif
#define FUSION_MAX_VECTOR_SIZE 15 //maximum number of statevariables (can be set to a larger value)
#include <boost/lexical_cast.hpp>
#include <boost/static_assert.hpp>
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

//this namespace contains some metaprogramming tools
namespace msf_tmp {

/**
 * \brief output a compile time constant by overflowing a char, producing a warning
 */
template<size_t size>
struct overflow {
  operator char() {
    return size + 9999;
  }
};
//always overflow, also for negative values
/**
 * \brief output a compile time constant
 */
template<int VALUE>
void echoCompileTimeConstant() {
  char(overflow<VALUE>());
  return;
}

/**
 * \brief runtime output of stateVariable types
 */
template<typename T> struct echoStateVarType;

/**
 * \brief runtime output of stateVariable types for const ref eigen matrices
 */
template<int NAME, int N, int STATE_T, int OPTIONS>
struct echoStateVarType<
    const msf_core::StateVar_T<Eigen::Matrix<double, N, 1>, NAME, STATE_T,
        OPTIONS>&> {
  static std::string value() {
    return "const ref Eigen::Matrix<double, "
        + boost::lexical_cast<std::string>(N) + ", 1>";
  }
};
/**
 * \brief runtime output of stateVariable types for const ref eigen quaternions
 */
template<int NAME, int STATE_T, int OPTIONS>
struct echoStateVarType<
    const msf_core::StateVar_T<Eigen::Quaterniond, NAME, STATE_T, OPTIONS>&> {
  static std::string value() {
    return "const ref Eigen::Quaterniond";
  }
};
/**
 * \brief runtime output of stateVariable types for eigen matrices
 */
template<int NAME, int N, int STATE_T, int OPTIONS>
struct echoStateVarType<
    msf_core::StateVar_T<Eigen::Matrix<double, N, 1>, NAME, STATE_T, OPTIONS> > {
  static std::string value() {
    return "Eigen::Matrix<double, " + boost::lexical_cast<std::string>(N)
        + ", 1>";
  }
};
/**
 * \brief runtime output of stateVariable types for eigen matrices
 */
template<int NAME, int STATE_T, int OPTIONS>
struct echoStateVarType<
    msf_core::StateVar_T<Eigen::Quaterniond, NAME, STATE_T, OPTIONS> > {
  static std::string value() {
    return "Eigen::Quaterniond";
  }
};

namespace {  //for internal use only
//the number of entries in the correction vector for a given state var
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

//the number of entries in the state for a given state var
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

//the number of entries in the state for a given state var if it is core state
template<typename T>
struct CoreStateLengthForType;
template<int NAME, int N, int STATE_T, int OPTIONS>
struct CoreStateLengthForType<
    const msf_core::StateVar_T<Eigen::Matrix<double, N, 1>, NAME, STATE_T,
        OPTIONS>&> {
  enum {
    value = 0
  };
  //not a core state, so length is zero
};
template<int NAME, int STATE_T, int OPTIONS>
struct CoreStateLengthForType<
    const msf_core::StateVar_T<Eigen::Quaterniond, NAME, STATE_T, OPTIONS>&> {
  enum {
    value = 0
  };
  //not a core state, so length is zero
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

//the number of entries in the state for a given state var if it is core state with propagation
template<typename T>
struct PropagatedCoreStateLengthForType;
template<int NAME, int N, int STATE_T, int OPTIONS>
struct PropagatedCoreStateLengthForType<
    const msf_core::StateVar_T<Eigen::Matrix<double, N, 1>, NAME, STATE_T,
        OPTIONS>&> {
  enum {
    value = 0
  };
  //not a core state, so length is zero
};
template<int NAME, int STATE_T, int OPTIONS>
struct PropagatedCoreStateLengthForType<
    const msf_core::StateVar_T<Eigen::Quaterniond, NAME, STATE_T, OPTIONS>&> {
  enum {
    value = 0
  };
  //not a core state, so length is zero
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

//the number of entries in the error state for a given state var if it is core state with propagation
template<typename T>
struct PropagatedCoreErrorStateLengthForType;
template<int NAME, int N, int STATE_T, int OPTIONS>
struct PropagatedCoreErrorStateLengthForType<
    const msf_core::StateVar_T<Eigen::Matrix<double, N, 1>, NAME, STATE_T,
        OPTIONS>&> {
  enum {
    value = 0
  };
  //not a core state, so length is zero
};
template<int NAME, int STATE_T, int OPTIONS>
struct PropagatedCoreErrorStateLengthForType<
    const msf_core::StateVar_T<Eigen::Quaterniond, NAME, STATE_T, OPTIONS>&> {
  enum {
    value = 0
  };
  //not a core state, so length is zero
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

//return the number a state has in the enum
template<typename T>
struct getEnumStateName;
template<typename U, int NAME, int STATE_T, int OPTIONS>
struct getEnumStateName<const msf_core::StateVar_T<U, NAME, STATE_T, OPTIONS>&> {
  enum {
    value = NAME
  };
};
template<typename U, int NAME, int STATE_T, int OPTIONS>
struct getEnumStateName<msf_core::StateVar_T<U, NAME, STATE_T, OPTIONS> > {
  enum {
    value = NAME
  };
};
template<> struct getEnumStateName<const mpl_::void_&> {
  enum {
    value = -1
  };
  //must not change this
};
template<> struct getEnumStateName<mpl_::void_> {
  enum {
    value = -1
  };
  //must not change this
};

template<typename T>
struct isQuaternionType;
template<int NAME, int STATE_T, int OPTIONS, int M, int N>
struct isQuaternionType<
    const msf_core::StateVar_T<Eigen::Matrix<double, M, N>, NAME, STATE_T,
        OPTIONS>&> {
  enum {
    value = false
  };
};
template<int NAME, int STATE_T, int OPTIONS, int M, int N>
struct isQuaternionType<
    msf_core::StateVar_T<Eigen::Matrix<double, M, N>, NAME, STATE_T, OPTIONS> > {
  enum {
    value = false
  };
};
template<int NAME, int STATE_T, int OPTIONS>
struct isQuaternionType<
    const msf_core::StateVar_T<Eigen::Quaterniond, NAME, STATE_T, OPTIONS>&> {
  enum {
    value = true
  };
};
template<int NAME, int STATE_T, int OPTIONS>
struct isQuaternionType<
    msf_core::StateVar_T<Eigen::Quaterniond, NAME, STATE_T, OPTIONS> > {
  enum {
    value = true
  };
};
template<>
struct isQuaternionType<const mpl_::void_&> {
  enum {
    value = false
  };
};
template<>
struct isQuaternionType<mpl_::void_> {
  enum {
    value = false
  };
};

/**
 * \brief return whether a state is nontemporaldrifting
 */
template<typename T>
struct getStateIsNonTemporalDrifting;
template<typename U, int NAME, int STATE_T, int OPTIONS>
struct getStateIsNonTemporalDrifting<
    const msf_core::StateVar_T<U, NAME, STATE_T, OPTIONS>&> {
  enum {
    value = static_cast<int>(msf_core::StateVar_T<U, NAME, STATE_T, OPTIONS>::statetype_)
        == static_cast<int>(msf_core::AuxiliaryNonTemporalDrifting)
  };
};
template<typename U, int NAME, int STATE_T, int OPTIONS>
struct getStateIsNonTemporalDrifting<
    msf_core::StateVar_T<U, NAME, STATE_T, OPTIONS> > {
  enum {
    value = static_cast<int>(msf_core::StateVar_T<U, NAME, STATE_T, OPTIONS>::statetype_)
        == static_cast<int>(msf_core::AuxiliaryNonTemporalDrifting)
  };
};
template<> struct getStateIsNonTemporalDrifting<const mpl_::void_&> {
  enum {
    value = false
  };
};
template<> struct getStateIsNonTemporalDrifting<mpl_::void_> {
  enum {
    value = false
  };
};

/**
 * \brief count some entity in the state vector using the counter class provided
 */
template<typename Sequence, template<typename > class Counter, typename First,
    typename Last, bool T>
struct countStatesLinear;
template<typename Sequence, template<typename > class Counter, typename First,
    typename Last>
struct countStatesLinear<Sequence, Counter, First, Last, true> {
  enum {  //the end does not add entries
    value = 0
  };
};
template<typename Sequence, template<typename > class Counter, typename First,
    typename Last>
struct countStatesLinear<Sequence, Counter, First, Last, false> {
  typedef typename boost::fusion::result_of::next<First>::type Next;
  typedef typename boost::fusion::result_of::deref<First>::type current_Type;
  enum {  //the length of the current state plus the tail of the list
    value = Counter<current_Type>::value
        + countStatesLinear<Sequence, Counter, Next, Last,
            SameType<typename msf_tmp::StripConstReference<First>::result_t,
                typename msf_tmp::StripConstReference<Last>::result_t>::value>::value
  };
};

/**
 * \brief helpers for CheckCorrectIndexing call{
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
  enum {  //the length of the current state plus the tail of the list
    idxInEnum = boost::mpl::if_c<getEnumStateName<current_Type>::value != -1,  //only evaluate if not at the end of the list
    boost::mpl::int_<getEnumStateName<current_Type>::value>,  //if we're not at the end of the list, use the calculated enum index
        boost::mpl::int_<CurrentIdx> >::type::value,  //else use the current index, so the assertion doesn't fail

    idxInState = CurrentIdx,

    indexingerrors = boost::mpl::if_c<idxInEnum == idxInState,
        boost::mpl::int_<0>, boost::mpl::int_<1> >::type::value +  //error here
        CheckStateIndexing<Sequence, Next, Last, CurrentIdx + 1, 	//and errors of other states
            SameType<typename msf_tmp::StripConstReference<First>::result_t,
                typename msf_tmp::StripConstReference<Last>::result_t>::value>::indexingerrors

  };
 private:
  //Error the ordering in the enum defining the names of the states is not the same ordering as in the type vector for the states
  BOOST_STATIC_ASSERT_MSG(indexingerrors==0, "Error the ordering in the enum defining the names _ "
      "of the states is not the same ordering as in the type vector for the states");
};
//}

/**
 * \brief return the state type of a given enum value
 */
template<typename TypeList, int INDEX>
struct getEnumStateType {
  typedef typename boost::fusion::result_of::at_c<TypeList, INDEX>::type value;
};

/**
 * \brief return void type if index is -1
 */
template<typename TypeList>
struct getEnumStateType<TypeList, -1> {
  typedef mpl_::void_ value;
};

/**
 * \brief helper for getStartIndex{
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
  enum { 		//return error code if end of list reached and type not found
    value = -1
  };
};
template<typename Sequence, typename StateVarT,
    template<typename > class OffsetCalculator, typename First, typename Last,
    bool TypeFound, int CurrentVal>
struct ComputeStartIndex<Sequence, StateVarT, OffsetCalculator, First, Last,
    TypeFound, CurrentVal, false> {
  enum { 		//we found the type, do not add additional offset
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

  enum { 		//the length of the current state plus the tail of the list
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
 * \brief helper for find best nontemporal drifting state{
 */
template<typename Sequence, typename First, typename Last, int CurrentBestIdx,
    bool foundQuaternion, bool EndOfList>
struct FindBestNonTemporalDriftingStateImpl;
template<typename Sequence, typename First, typename Last, int CurrentBestIdx,
    bool foundQuaternion>
struct FindBestNonTemporalDriftingStateImpl<Sequence, First, Last,
    CurrentBestIdx, foundQuaternion, true> {
  enum { 		//set to the current index found, which was initially set to -1
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
    idxInState = getEnumStateName<
        typename msf_tmp::StripConstReference<current_Type>::result_t>::value,  //we assert before calling that the indexing is correct
    IsCurrentStateAQuaternion = isQuaternionType<
        typename msf_tmp::StripConstReference<current_Type>::result_t>::value,
    IsCurrentStateNonTemporalDrifting = getStateIsNonTemporalDrifting<
        typename msf_tmp::StripConstReference<current_Type>::result_t>::value,
    isQuaternionAndNonTemporalDrifting = IsCurrentStateAQuaternion == true
        && IsCurrentStateNonTemporalDrifting == true,
    isNonTemporalDriftingPositionAndNoQuaternionHasBeenFound = IsCurrentStateNonTemporalDrifting
        == true && foundQuaternion == false,

    currBestIdx = boost::mpl::if_c<isQuaternionAndNonTemporalDrifting,
        boost::mpl::int_<idxInState>,  //set to this index if quaternion and nontemporal drifting
        boost::mpl::int_<
            boost::mpl::if_c<
                isNonTemporalDriftingPositionAndNoQuaternionHasBeenFound,
                boost::mpl::int_<idxInState>,  //if we havent found a quaternion yet and this state is non temporal drifting, we take this one
                boost::mpl::int_<CurrentBestIdx> >::type::value>  //else we take the current best one
    >::type::value,

    value = FindBestNonTemporalDriftingStateImpl<Sequence, Next, Last,
        currBestIdx, foundQuaternion || isQuaternionAndNonTemporalDrifting,
        SameType<typename msf_tmp::StripConstReference<First>::result_t,
            typename msf_tmp::StripConstReference<Last>::result_t>::value>::value

  };
};

}  //end anonymous namespace

/**
 * \brief checks whether the ordering in the vector is the same as in the enum
 *
 * this ordering is something that strictly must not change
 */
template<typename Sequence>
struct CheckCorrectIndexing {
  typedef typename boost::fusion::result_of::begin<Sequence const>::type First;
  typedef typename boost::fusion::result_of::end<Sequence const>::type Last;
 private:
  enum {
    startindex = 0,  //must not change this
  };
 public:
  enum {
    indexingerrors = CheckStateIndexing<Sequence, First, Last, startindex,
        SameType<typename msf_tmp::StripConstReference<First>::result_t,
            typename msf_tmp::StripConstReference<Last>::result_t>::value>::indexingerrors
  };
};

/**
 * \brief find the best nontemporal drifting state in the full state vector at compile time
 *
 * the goal of this tmp code is to find the best nontemporal drifting state in the
 * full state vector at compile time. We therefore search the state variable list
 * for states that the user has marked as non temporal drifting. If furthermore
 * prefer quaternions over euclidean states. This function will return -1 if no suitable state has been found
 */
template<typename Sequence>
struct IndexOfBestNonTemporalDriftingState {
  typedef typename boost::fusion::result_of::begin<Sequence const>::type First;
  typedef typename boost::fusion::result_of::end<Sequence const>::type Last;
 private:
  enum {
    bestindex = -1,  //must not change this
  };
  BOOST_STATIC_ASSERT_MSG(CheckCorrectIndexing<Sequence>::indexingerrors == 0, "The indexing of the state vector is not the same as in the enum,"
      " but this must be the same");
public:
  enum {
    value = FindBestNonTemporalDriftingStateImpl<Sequence, First, Last, bestindex, false,
    SameType<typename msf_tmp::StripConstReference<First>::result_t,
    typename msf_tmp::StripConstReference<Last>::result_t>::value>::value
  };
};

/**
 * \brief returns the number of doubles in the state/correction state
 * depending on the counter type supplied
 */
template<typename Sequence, template<typename > class Counter>
struct CountStates {
  typedef typename boost::fusion::result_of::begin<Sequence const>::type First;
  typedef typename boost::fusion::result_of::end<Sequence const>::type Last;
  enum {
    value = countStatesLinear<Sequence, Counter, First, Last,
        SameType<typename msf_tmp::StripConstReference<First>::result_t,
            typename msf_tmp::StripConstReference<Last>::result_t>::value>::value
        + CheckCorrectIndexing<Sequence>::indexingerrors  //will be zero, if no indexing errors, otherwise fails compilation
  };
};

/**
 * \brief compute start indices in the correction/state vector of a given type
 */
template<typename Sequence, typename StateVarT,
    template<typename > class Counter>
struct getStartIndex {
  typedef typename boost::fusion::result_of::begin<Sequence const>::type First;
  typedef typename boost::fusion::result_of::end<Sequence const>::type Last;
  typedef typename boost::fusion::result_of::deref<First>::type currentType;
  enum {
    value = ComputeStartIndex<Sequence, StateVarT, Counter, First, Last,
        SameType<typename msf_tmp::StripConstReference<StateVarT>::result_t,
            typename msf_tmp::StripConstReference<currentType>::result_t>::value,
        0, SameType<First, Last>::value>::value
        + CheckCorrectIndexing<Sequence>::indexingerrors  //will be zero, if no indexing errors, otherwise fails compilation
  };
};

/**
 * \brief compute start indices in the correction vector of a given type
 */
template<typename Sequence, int StateEnum>
struct getStartIndexInCorrection {
  typedef typename boost::fusion::result_of::begin<Sequence const>::type First;
  typedef typename boost::fusion::result_of::end<Sequence const>::type Last;
  typedef typename boost::fusion::result_of::deref<First>::type currentType;
  enum {
    value = msf_tmp::getStartIndex<Sequence,
        typename msf_tmp::getEnumStateType<Sequence, StateEnum>::value,
        msf_tmp::CorrectionStateLengthForType>::value
  };
};

/**
 * \brief reset the EKF state in a boost fusion unrolled call
 */
struct resetState {
  template<int NAME, int N, int STATE_T, int OPTIONS>
  void operator()(
      msf_core::StateVar_T<Eigen::Matrix<double, N, 1>, NAME, STATE_T, OPTIONS>& t) const {
    typedef msf_core::StateVar_T<Eigen::Matrix<double, N, 1>, NAME, STATE_T,
        OPTIONS> var_T;
    t.state_.setZero();
  }
  template<int NAME, int STATE_T, int OPTIONS>
  void operator()(
      msf_core::StateVar_T<Eigen::Quaterniond, NAME, STATE_T, OPTIONS>& t) const {
    typedef msf_core::StateVar_T<Eigen::Quaterniond, NAME, STATE_T, OPTIONS> var_T;
    t.state_.setIdentity();
  }
};

/**
 * \brief copy states from previous to current states, for which there is no propagation
 * in a boost fusion unrolled call
 */
template<typename stateVarT>
struct copyInitStates {
  copyInitStates(const stateVarT& oldstate)
      : oldstate_(oldstate) {
  }
  template<typename T, int NAME, int STATE_T, int OPTIONS>
  void operator()(msf_core::StateVar_T<T, NAME, STATE_T, OPTIONS>& t) const {
    if (oldstate_.template getStateVar<NAME>().hasResetValue) {
      t = oldstate_.template getStateVar<NAME>();  //copy value from old state to new state var
      }
    }

  private:
    const stateVarT& oldstate_;
  };

    /**
     * \brief copy states from previous to current states, for which there is no propagation
     */
template<typename stateVarT>
struct copyNonPropagationStates {
  copyNonPropagationStates(const stateVarT& oldstate)
      : oldstate_(oldstate) {
  }
  template<typename T, int NAME, int OPTIONS>
  void operator()(
      msf_core::StateVar_T<T, NAME, msf_core::CoreStateWithPropagation, OPTIONS>& UNUSEDPARAM(t)) const {
    //nothing to do for the states, which have propagation
  }
  template<typename T, int NAME, int STATE_T, int OPTIONS>
  void operator()(msf_core::StateVar_T<T, NAME, STATE_T, OPTIONS>& t) const {

    BOOST_STATIC_ASSERT_MSG(
        (msf_tmp::IsPointerType<typename msf_core::StateVar_T<T, NAME, STATE_T> >::value == false),
        "The state variable is of pointer type, but this method assumes we can copy without dereferencing");

    BOOST_STATIC_ASSERT_MSG(
        STATE_T != msf_core::CoreStateWithPropagation,
        "copyNonPropagationStates was instantiated for a core state. This is an error.");

    t = oldstate_.template getStateVar<NAME>();  //copy value from old state to new state var
  }

 private:
  const stateVarT& oldstate_;
};

/**
 * \brief copy the user calculated values in the Q-blocks to the main Q matrix
 */
template<typename stateList_T>
struct copyQBlocksFromAuxiliaryStatesToQ {
  enum {
    nErrorStatesAtCompileTime = msf_tmp::CountStates<stateList_T,
        msf_tmp::CorrectionStateLengthForType>::value  //n correction states
  };
  typedef Eigen::Matrix<double, nErrorStatesAtCompileTime,
      nErrorStatesAtCompileTime> Q_T;
  copyQBlocksFromAuxiliaryStatesToQ(Q_T& Q)
      : Q_(Q) {
  }
  template<typename T, int NAME, int STATE_T, int OPTIONS>
  void operator()(msf_core::StateVar_T<T, NAME, STATE_T, OPTIONS>& t) const {
    typedef msf_core::StateVar_T<T, NAME, STATE_T, OPTIONS> var_T;
    enum {
      startIdxInCorrection = msf_tmp::getStartIndex<stateList_T, var_T,
          msf_tmp::CorrectionStateLengthForType>::value,
      sizeInCorrection = var_T::sizeInCorrection_
    };

    BOOST_STATIC_ASSERT_MSG(
        static_cast<int>(var_T::statetype_) != static_cast<int>(msf_core::CoreStateWithPropagation) && static_cast<int>(var_T::statetype_) != static_cast<int>(msf_core::CoreStateWithoutPropagation),
        "copyQBlocksFromAuxiliaryStatesToQ was instantiated for a core state. This is an error.");

    BOOST_STATIC_ASSERT_MSG(
        static_cast<int>(sizeInCorrection) == static_cast<int>(var_T::Q_T::ColsAtCompileTime) && static_cast<int>(sizeInCorrection) == static_cast<int>(var_T::Q_T::RowsAtCompileTime),
        "copyQBlocksFromAuxiliaryStatesToQ size of Matrix Q stored with the stateVar,"
        "is not the same as the reported dimension in the correction vector");

    Q_.template block<sizeInCorrection, sizeInCorrection>(startIdxInCorrection,
                                                          startIdxInCorrection) =
        t.Q;
  }
  template<typename T, int NAME, int OPTIONS>
  void operator()(
      msf_core::StateVar_T<T, NAME, msf_core::CoreStateWithPropagation, OPTIONS>& UNUSEDPARAM(t)) const {
    //nothing to do for the states, which have propagation, because Q calculation is done in msf_core
  }
  template<typename T, int NAME, int OPTIONS>
  void operator()(
      msf_core::StateVar_T<T, NAME, msf_core::CoreStateWithoutPropagation,
          OPTIONS>& UNUSEDPARAM(t)) const {
    //nothing to do for the states, which have propagation, because Q calculation is done in msf_core
  }
 private:
  Q_T& Q_;
};

/**
 * \brief apply EKF corrections depending on the stateVar type
 */
template<typename T, typename stateList_T>
struct correctState {
  correctState(T& correction)
      : data_(correction) {
  }
  template<int NAME, int N, int STATE_T, int OPTIONS>
  void operator()(
      msf_core::StateVar_T<Eigen::Matrix<double, N, 1>, NAME, STATE_T, OPTIONS>& t) const {
    typedef msf_core::StateVar_T<Eigen::Matrix<double, N, 1>, NAME, STATE_T,
        OPTIONS> var_T;
    enum {
      startIdxInCorrection = msf_tmp::getStartIndex<stateList_T, var_T,
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
      startIdxInCorrection = msf_tmp::getStartIndex<stateList_T, var_T,
          msf_tmp::CorrectionStateLengthForType>::value
    };
    //make sure the user does not define bogus options
    BOOST_STATIC_ASSERT_MSG(
        (OPTIONS & msf_core::correctionMultiplicative) == false,
        "You defined the Quaternion correction to be multiplicative, but this is anyway done and not an option");

    Eigen::Quaternion<double> qbuff_q = quaternionFromSmallAngle(
        data_.template block<var_T::sizeInCorrection_, 1>(startIdxInCorrection,
                                                          0));
    t.state_ = t.state_ * qbuff_q;
    t.state_.normalize();
  }
 private:
  T& data_;
};

/**
 * \brief copies the values of the single state vars to the double array provided
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
      startIdxInState = msf_tmp::getStartIndex<stateList_T, var_T,
          msf_tmp::StateLengthForType>::value  //index of the data in the state vector
    };
    BOOST_STATIC_ASSERT_MSG(
        static_cast<int>(var_T::statetype_) == static_cast<int>(msf_core::CoreStateWithPropagation) || static_cast<int>(var_T::statetype_) == static_cast<int>(msf_core::CoreStateWithoutPropagation),
        "CoreStatetoDoubleArray: A new statetype was defined, but I don't know whether this should be written to the double array for core states");
    for (int i = 0; i < var_T::sizeInState_; ++i) {
      data_[startIdxInState + i] = t.state_[i];
    }
  }
  template<int NAME, int STATE_T, int OPTIONS>
  void operator()(
      msf_core::StateVar_T<Eigen::Quaterniond, NAME, STATE_T, OPTIONS>& t) const {
    typedef msf_core::StateVar_T<Eigen::Quaterniond, NAME, STATE_T, OPTIONS> var_T;
    enum {
      startIdxInState = msf_tmp::getStartIndex<stateList_T, var_T,
          msf_tmp::StateLengthForType>::value  //index of the data in the state vector
    };
    BOOST_STATIC_ASSERT_MSG(
        static_cast<int>(var_T::statetype_) == static_cast<int>(msf_core::CoreStateWithPropagation) || static_cast<int>(var_T::statetype_) == static_cast<int>(msf_core::CoreStateWithoutPropagation),
        "CoreStatetoDoubleArray: A new statetype was defined, but I don't know whether this should be written to the double array for core states");
    //copy quaternion values
    data_[startIdxInState + 0] = t.state_.w();
    data_[startIdxInState + 1] = t.state_.x();
    data_[startIdxInState + 2] = t.state_.y();
    data_[startIdxInState + 3] = t.state_.z();
  }
  template<int NAME, int OPTIONS, int N>
  void operator()(
      msf_core::StateVar_T<Eigen::Matrix<double, N, 1>, NAME,
          msf_core::Auxiliary, OPTIONS>& UNUSEDPARAM(t)) const {
    //don't copy aux states
  }
  template<int NAME, int OPTIONS>
  void operator()(
      msf_core::StateVar_T<Eigen::Quaterniond, NAME, msf_core::Auxiliary,
          OPTIONS>& UNUSEDPARAM(t)) const {
    //don't copy aux states
  }

  template<int NAME, int OPTIONS, int N>
  void operator()(
      msf_core::StateVar_T<Eigen::Matrix<double, N, 1>, NAME,
          msf_core::AuxiliaryNonTemporalDrifting, OPTIONS>& UNUSEDPARAM(t)) const {
    //don't copy aux states
  }
  template<int NAME, int OPTIONS>
  void operator()(
      msf_core::StateVar_T<Eigen::Quaterniond, NAME,
          msf_core::AuxiliaryNonTemporalDrifting, OPTIONS>& UNUSEDPARAM(t)) const {
    //don't copy aux states
  }
 private:
  T& data_;
};

/**
 * \brief copies pairs of statename and index in correction vector to STL container of pairs or tuples
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
      startIdxInState = msf_tmp::getStartIndex<stateList_T, var_T,
          msf_tmp::CorrectionStateLengthForType>::value,  //index of the data in the correction vector
      lengthInState = var_T::sizeInCorrection_
    };
    data_.push_back(
        typename T::value_type(NAME, startIdxInState, lengthInState));
  }
 private:
  T& data_;
};

/**
 * \brief return the index of a given state variable number in the state matrices
 */
template<typename stateVector_T, int INDEX>
struct getStateIndexInState {
  typedef typename msf_tmp::getEnumStateType<stateVector_T, INDEX>::value state_type;
  enum {
    value = msf_tmp::getStartIndex<stateVector_T, state_type,
        msf_tmp::StateLengthForType>::value,
  };
};

/**
 * \brief return the index of a given state variable number in the error state matrices
 */
template<typename stateVector_T, int INDEX>
struct getStateIndexInErrorState {
  typedef typename msf_tmp::getEnumStateType<stateVector_T, INDEX>::value state_type;
  enum {
    value = msf_tmp::getStartIndex<stateVector_T, state_type,
        msf_tmp::CorrectionStateLengthForType>::value,
  };
};

/**
 * \brief copies the values of the single state vars to the string provided
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
      startIdxInState = msf_tmp::getStartIndex<stateList_T, var_T,
          msf_tmp::StateLengthForType>::value  //index of the data in the state vector
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
      startIdxInState = msf_tmp::getStartIndex<stateList_T, var_T,
          msf_tmp::StateLengthForType>::value  //index of the data in the state vector
    };
    data_ << NAME << " : [" << startIdxInState << "-" << startIdxInState + 3
        << "]\t : Quaternion (w,x,y,z) : " << STREAMQUAT(t.state_) << std::endl;
  }
 private:
  STREAM& data_;
};

/**
 * \brief copies the values of the single state vars to the double array provided
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
      startIdxInState = msf_tmp::getStartIndex<stateList_T, var_T,
          msf_tmp::StateLengthForType>::value  //index of the data in the state vector
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
      startIdxInState = msf_tmp::getStartIndex<stateList_T, var_T,
          msf_tmp::StateLengthForType>::value  //index of the data in the state vector
    };
    //copy quaternion values
    data_[startIdxInState + 0] = t.state_.w();
    data_[startIdxInState + 1] = t.state_.x();
    data_[startIdxInState + 2] = t.state_.y();
    data_[startIdxInState + 3] = t.state_.z();
  }
 private:
  T& data_;
};
}

#endif /* MSF_TMP_HPP_ */
