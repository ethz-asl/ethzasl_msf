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

#ifndef STATIC_ORDERING_ASSERTIONS_CORE_H_
#define STATIC_ORDERING_ASSERTIONS_CORE_H_

#include <boost/static_assert.hpp>
#include <msf_core/msf_tmp.h>

namespace {

template<typename EKFState_T>
struct StaticAssertCoreStateOrderingCorrect {
  typedef typename EKFState_T::StateSequence_T StateSequence_T;
  typedef typename EKFState_T::StateDefinition_T StateDefinition_T;

  enum {
    value
  };

  // For now we have no make sure, the core states are in the correct order
  //(calculation of observation noise cov has hardcoded order)
  // DO NOT REMOVE THIS!! UNLESS YOU ALSO FIXED CalcQCore!
  // TODO: (slynen) This should be ok now, apart from the Cov init matrix.
  //{
  enum {
    idxstartcorr_p_ = msf_tmp::getStartIndex<StateSequence_T,
        typename msf_tmp::getEnumStateType<StateSequence_T, StateDefinition_T::p>::value,
        msf_tmp::CorrectionStateLengthForType>::value,

    idxstartcorr_v_ = msf_tmp::getStartIndex<StateSequence_T,
        typename msf_tmp::getEnumStateType<StateSequence_T, StateDefinition_T::v>::value,
        msf_tmp::CorrectionStateLengthForType>::value,

    idxstartcorr_q_ = msf_tmp::getStartIndex<StateSequence_T,
        typename msf_tmp::getEnumStateType<StateSequence_T, StateDefinition_T::q>::value,
        msf_tmp::CorrectionStateLengthForType>::value,

    idxstartcorr_b_w_ = msf_tmp::getStartIndex<StateSequence_T,
        typename msf_tmp::getEnumStateType<StateSequence_T,
            StateDefinition_T::b_w>::value,
        msf_tmp::CorrectionStateLengthForType>::value,

    idxstartcorr_b_a_ = msf_tmp::getStartIndex<StateSequence_T,
        typename msf_tmp::getEnumStateType<StateSequence_T,
            StateDefinition_T::b_a>::value,
        msf_tmp::CorrectionStateLengthForType>::value
  };

  static_assert(idxstartcorr_p_==0, "Indexing of core states has been altered, "
      "but this is currently not allowed");
  static_assert(idxstartcorr_v_==3, "Indexing of core states has been altered, "
      "but this is currently not allowed");
  static_assert(idxstartcorr_q_==6, "Indexing of core states has been altered, "
      "but this is currently not allowed");
  static_assert(idxstartcorr_b_w_==9, "Indexing of core states has been altered, "
      "but this is currently not allowed");
  static_assert(idxstartcorr_b_a_==12, "Indexing of core states has been altered, "
      "but this is currently not allowed");
  //}
};

}

#endif  // STATIC_ORDERING_ASSERTIONS_H_
