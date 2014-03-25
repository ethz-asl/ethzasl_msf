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

#include <msf_core/msf_core.h>
#include <msf_core/testing_entrypoint.h>

// Test calculated sizes.
TEST(MSF_Core, CompileTimeComputation_StateSizeCalculation) {
  using namespace msf_core;
  enum StateDefinition {
    a,
    b,
    c,
    d
  };
  const static int vectorlength1 = 4;
  const static int vectorlength2 = 10;

  // Setup some state type.
  typedef boost::fusion::vector<
      msf_core::StateVar_T<Eigen::Matrix<double, vectorlength1, 1>, a>,
      msf_core::StateVar_T<Eigen::Quaterniond, b>,
      msf_core::StateVar_T<Eigen::Matrix<double, 1, 1>, c>,
      msf_core::StateVar_T<Eigen::Matrix<double, vectorlength2, 1>, d>
  > fullState_T;
  typedef msf_core::GenericState_T<fullState_T, StateDefinition> EKFState;

  const EKFState somestate;

  EXPECT_EQ(somestate.GetStateVariable<0>().sizeInCorrection_, vectorlength1);
  EXPECT_EQ(somestate.GetStateVariable<1>().sizeInCorrection_, 3);
  EXPECT_EQ(somestate.GetStateVariable<2>().sizeInCorrection_, 1);
  EXPECT_EQ(somestate.GetStateVariable<3>().sizeInCorrection_, vectorlength2);

  EXPECT_EQ(somestate.GetStateVariable<0>().sizeInState_, vectorlength1);
  EXPECT_EQ(somestate.GetStateVariable<1>().sizeInState_, 4);
  EXPECT_EQ(somestate.GetStateVariable<2>().sizeInState_, 1);
  EXPECT_EQ(somestate.GetStateVariable<3>().sizeInState_, vectorlength2);
}

// Test indices of statevars in state vectors.
TEST(MSF_Core, CompileTimeComputation_StateIndexCalculation) {
  using namespace msf_core;
  enum StateDefinition {
    a,
    b,
    c,
    d
  };
  const static int vectorlength1 = 4;
  const static int vectorlength2 = 10;

  // Setup some state type.
  typedef boost::fusion::vector<
      StateVar_T<Eigen::Matrix<double, vectorlength1, 1>, a>,
      StateVar_T<Eigen::Quaterniond, b>,
      StateVar_T<Eigen::Matrix<double, 1, 1>, c>,
      StateVar_T<Eigen::Matrix<double, vectorlength2, 1>, d>
  > fullState_T;
  typedef GenericState_T<fullState_T, StateDefinition> EKFState;

  EKFState somestate;
  static const int idxstartcorr1 = msf_tmp::GetStartIndex<fullState_T,
      StateVar_T<Eigen::Matrix<double, vectorlength1, 1>, a>,
      msf_tmp::CorrectionStateLengthForType>::value;
  static const int idxstartstate1 = msf_tmp::GetStartIndex<fullState_T,
      StateVar_T<Eigen::Matrix<double, vectorlength1, 1>, a>,
      msf_tmp::StateLengthForType>::value;
  EXPECT_EQ(idxstartcorr1, 0);
  EXPECT_EQ(idxstartstate1, 0);

  static const int idxstartcorr2 = msf_tmp::GetStartIndex<fullState_T,
      StateVar_T<Eigen::Quaterniond, b>,
      msf_tmp::CorrectionStateLengthForType>::value;
  static const int idxstartstate2 = msf_tmp::GetStartIndex<fullState_T,
      StateVar_T<Eigen::Quaterniond, b>, msf_tmp::StateLengthForType>::value;
  EXPECT_EQ(idxstartcorr2, vectorlength1);
  EXPECT_EQ(idxstartstate2, vectorlength1);

  static const int idxstartcorr3 = msf_tmp::GetStartIndex<fullState_T,
      StateVar_T<Eigen::Matrix<double, 1, 1>, c>,
      msf_tmp::CorrectionStateLengthForType>::value;
  static const int idxstartstate3 = msf_tmp::GetStartIndex<fullState_T,
      StateVar_T<Eigen::Matrix<double, 1, 1>, c>,
      msf_tmp::StateLengthForType>::value;
  EXPECT_EQ(idxstartcorr3, vectorlength1 + 3);
  EXPECT_EQ(idxstartstate3, vectorlength1 + 4);

  static const int idxstartcorr4 = msf_tmp::GetStartIndex<fullState_T,
      StateVar_T<Eigen::Matrix<double, vectorlength2, 1>, d>,
      msf_tmp::CorrectionStateLengthForType>::value;
  static const int idxstartstate4 = msf_tmp::GetStartIndex<fullState_T,
      StateVar_T<Eigen::Matrix<double, vectorlength2, 1>, d>,
      msf_tmp::StateLengthForType>::value;
  EXPECT_EQ(idxstartcorr4, vectorlength1 + 3 + 1);
  EXPECT_EQ(idxstartstate4, vectorlength1 + 4 + 1);

}

// Tests compile time computed values for the state.
TEST(MSF_Core, CompileTimeComputation_StateLengthCalculation) {
  using namespace msf_core;
  enum StateDefinition {
    a,
    b,
    c,
    d
  };
  const static int vectorlength1 = 3;
  const static int vectorlength2 = 20;

  //setup some state type
  typedef boost::fusion::vector<
      StateVar_T<Eigen::Matrix<double, vectorlength1, 1>, a>,
      StateVar_T<Eigen::Quaterniond, b>,
      StateVar_T<Eigen::Matrix<double, 1, 1>, c>,
      StateVar_T<Eigen::Matrix<double, vectorlength2, 1>, d>
  > fullState_T;
  typedef GenericState_T<fullState_T, StateDefinition> EKFState;

  EXPECT_EQ(EKFState::nStateVarsAtCompileTime, 4);
  EXPECT_EQ(EKFState::nStatesAtCompileTime,
            vectorlength1 + vectorlength2 + 4 + 1);
  EXPECT_EQ(EKFState::nErrorStatesAtCompileTime,
            vectorlength1 + vectorlength2 + 3 + 1);
}

TEST(MSF_Core, RuntimeTimeComputation_CopyForNonPropagationStates) {
  enum StateDefinition {
    p_,
    v_,
    q_,
    b_w_,
    b_a_,
    L_,
    q_wv_,
    q_ci_,
    p_ci_
  };

  typedef boost::fusion::vector<
      msf_core::StateVar_T<Eigen::Matrix<double, 3, 1>, p_, true>,
      msf_core::StateVar_T<Eigen::Matrix<double, 3, 1>, v_, true>,
      msf_core::StateVar_T<Eigen::Quaternion<double>, q_, true>,
      msf_core::StateVar_T<Eigen::Matrix<double, 3, 1>, b_w_, true>,
      msf_core::StateVar_T<Eigen::Matrix<double, 3, 1>, b_a_, true>,

      msf_core::StateVar_T<Eigen::Matrix<double, 1, 1>, L_>,
      msf_core::StateVar_T<Eigen::Quaternion<double>, q_wv_>,
      msf_core::StateVar_T<Eigen::Quaternion<double>, q_ci_>,
      msf_core::StateVar_T<Eigen::Matrix<double, 3, 1>, p_ci_>
  > fullState_T;

  typedef msf_core::GenericState_T<fullState_T, StateDefinition> EKFState;

  EKFState first_state;
  EKFState second_state;

  first_state.Reset();
  second_state.Reset();

  Eigen::Matrix<double, 3, 1> p;
  p.setRandom();
  first_state.Set<p_>(p);

  Eigen::Quaternion<double> q;
  q.w() = 0.4;
  q.x() = 0.1;
  q.y() = 0.2;
  q.z() = 0.7;
  q.normalize();
  first_state.Set<q_>(q);

  boost::fusion::for_each(
      first_state.statevars,
      msf_tmp::CopyNonPropagationStates<EKFState>(second_state));

  const EKFState& const_ref_first_state = first_state;
  const EKFState& const_ref_second_state = second_state;

  EXPECT_DOUBLE_EQ(const_ref_first_state.Get<p_>()(0),
                   const_ref_second_state.Get<p_>()(0));
  EXPECT_DOUBLE_EQ(const_ref_first_state.Get<p_>()(1),
                   const_ref_second_state.Get<p_>()(1));
  EXPECT_DOUBLE_EQ(const_ref_first_state.Get<p_>()(2),
                   const_ref_second_state.Get<p_>()(2));

  EXPECT_DOUBLE_EQ(const_ref_first_state.Get<q_>().w(),
                   const_ref_second_state.Get<q_>().w());
  EXPECT_DOUBLE_EQ(const_ref_first_state.Get<q_>().x(),
                   const_ref_second_state.Get<q_>().x());
  EXPECT_DOUBLE_EQ(const_ref_first_state.Get<q_>().y(),
                   const_ref_second_state.Get<q_>().y());
  EXPECT_DOUBLE_EQ(const_ref_first_state.Get<q_>().z(),
                   const_ref_second_state.Get<q_>().z());
}

MSF_UNITTEST_ENTRYPOINT
