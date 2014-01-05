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
#ifndef STATIC_ORDERING_ASSERTIONS_CORE_H_
#define STATIC_ORDERING_ASSERTIONS_CORE_H_

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
    idxstartcorr_p_ = msf_tmp::GetStartIndex<StateSequence_T,
        typename msf_tmp::GetEnumStateType<StateSequence_T, StateDefinition_T::p>::value,
        msf_tmp::CorrectionStateLengthForType>::value,

    idxstartcorr_v_ = msf_tmp::GetStartIndex<StateSequence_T,
        typename msf_tmp::GetEnumStateType<StateSequence_T, StateDefinition_T::v>::value,
        msf_tmp::CorrectionStateLengthForType>::value,

    idxstartcorr_q_ = msf_tmp::GetStartIndex<StateSequence_T,
        typename msf_tmp::GetEnumStateType<StateSequence_T, StateDefinition_T::q>::value,
        msf_tmp::CorrectionStateLengthForType>::value,

    idxstartcorr_b_w_ = msf_tmp::GetStartIndex<StateSequence_T,
        typename msf_tmp::GetEnumStateType<StateSequence_T,
            StateDefinition_T::b_w>::value,
        msf_tmp::CorrectionStateLengthForType>::value,

    idxstartcorr_b_a_ = msf_tmp::GetStartIndex<StateSequence_T,
        typename msf_tmp::GetEnumStateType<StateSequence_T,
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
