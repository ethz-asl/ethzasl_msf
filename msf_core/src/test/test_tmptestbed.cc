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

#include <msf_core/msf_state.h>

int main(int argc, char** argv) {

  msf_core::EKFState somestate;
  msf_core::EKFState otherstate;

  //find best non temporal drifting state
  std::cout << "index of best non temporal drifting state "
      << msf_tmp::IndexOfBestNonTemporalDriftingState<msf_core::fullState_T>::value
      << std::endl;
  typedef typename msf_tmp::GetEnumStateType<msf_core::fullState_T,
      msf_tmp::IndexOfBestNonTemporalDriftingState<msf_core::fullState_T>::value>::value nonDriftingStateType;

  const bool isquaternion =
      msf_tmp::IsQuaternionType<
          typename msf_tmp::StripConstReference<nonDriftingStateType>::result_t>::value;

//
//	//qualifier stripping
//	typedef boost::fusion::vector<int,float,char> vec;
//	BOOST_MPL_ASSERT((boost::is_same<boost::fusion::result_of::at_c<vec, 1>::type, float&>));
//	BOOST_MPL_ASSERT((boost::is_same<msf_tmp::StripConstReference<boost::fusion::result_of::at_c<vec, 1>::type>::result_t, float>));
//
//	//counting and index calculation
//	static const int idxstartcorr_p_ = msf_tmp::GetStartIndex<msf_core::fullState_T, msf_tmp::GetEnumStateType<msf_core::fullState_T, msf_core::p_>::value, msf_tmp::CorrectionStateLengthForType>::value;
//	static const int idxstartcorr_v_ = msf_tmp::GetStartIndex<msf_core::fullState_T, msf_tmp::GetEnumStateType<msf_core::fullState_T, msf_core::v_>::value, msf_tmp::CorrectionStateLengthForType>::value;
//	static const int idxstartcorr_q_ = msf_tmp::GetStartIndex<msf_core::fullState_T, msf_tmp::GetEnumStateType<msf_core::fullState_T, msf_core::q_>::value, msf_tmp::CorrectionStateLengthForType>::value;
//	static const int idxstartcorr_b_w_ = msf_tmp::GetStartIndex<msf_core::fullState_T, msf_tmp::GetEnumStateType<msf_core::fullState_T, msf_core::b_w_>::value, msf_tmp::CorrectionStateLengthForType>::value;
//	static const int idxstartcorr_b_a_ = msf_tmp::GetStartIndex<msf_core::fullState_T, msf_tmp::GetEnumStateType<msf_core::fullState_T, msf_core::b_a_>::value, msf_tmp::CorrectionStateLengthForType>::value;
//
//	std::cout<<"idxstartcorr_p_ "<<idxstartcorr_p_<<std::endl;
//	std::cout<<"idxstartcorr_v_ "<<idxstartcorr_v_<<std::endl;
//	std::cout<<"idxstartcorr_q_ "<<idxstartcorr_q_<<std::endl;
//	std::cout<<"idxstartcorr_b_w_ "<<idxstartcorr_b_w_<<std::endl;
//	std::cout<<"idxstartcorr_b_a_ "<<idxstartcorr_b_a_<<std::endl;
//
//	Eigen::Matrix<double, 3, 1> v1;
//	v1<<1,2,4;
//
//	Eigen::Matrix<double, 3, 3> m;
//
//	m = (v1.cwiseProduct(v1)).asDiagonal();
//	std::cout<<(v1.cwiseProduct(v1))<<std::endl<<std::endl;
//	std::cout<<m<<std::endl;
//
//	Eigen::Matrix<double, 5, 5> n;
//
//	std::cout<<n<<std::endl;
//
//	Eigen::Matrix<double, 3,3> other;
////n(2,2)=other(0,0);
//	n.block<2,2>(2,2) = other.block<2,2>(0,0);
//
//	const msf_core::EKFState& stateref = somestate;
//
//	//std::cout<<
//			stateref.get<msf_core::p_>();
//	//<<std::endl;
//
//	std::cout<<n<<std::endl;

}

