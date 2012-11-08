/*
 * tmp_testbed.cc
 *
 *  Created on: Nov 7, 2012
 *      Author: slynen
 */

#include <msf_core/msf_state.hpp>
int main(int argc, char** argv)
{

	//qualifier stripping
	typedef boost::fusion::vector<int,float,char> vec;
	BOOST_MPL_ASSERT((boost::is_same<boost::fusion::result_of::at_c<vec, 1>::type, float&>));
	BOOST_MPL_ASSERT((boost::is_same<msf_tmp::StripConstReference<boost::fusion::result_of::at_c<vec, 1>::type>::value, float>));

	//counting and index calculation
	static const int idxstartcorr_p_ = msf_tmp::getStartIndex<msf_core::fullState_T, msf_tmp::getEnumStateType<msf_core::fullState_T, msf_core::p_>::value, msf_tmp::CorrectionStateLengthForType>::value;
	static const int idxstartcorr_v_ = msf_tmp::getStartIndex<msf_core::fullState_T, msf_tmp::getEnumStateType<msf_core::fullState_T, msf_core::v_>::value, msf_tmp::CorrectionStateLengthForType>::value;
	static const int idxstartcorr_q_ = msf_tmp::getStartIndex<msf_core::fullState_T, msf_tmp::getEnumStateType<msf_core::fullState_T, msf_core::q_>::value, msf_tmp::CorrectionStateLengthForType>::value;
	static const int idxstartcorr_b_w_ = msf_tmp::getStartIndex<msf_core::fullState_T, msf_tmp::getEnumStateType<msf_core::fullState_T, msf_core::b_w_>::value, msf_tmp::CorrectionStateLengthForType>::value;
	static const int idxstartcorr_b_a_ = msf_tmp::getStartIndex<msf_core::fullState_T, msf_tmp::getEnumStateType<msf_core::fullState_T, msf_core::b_a_>::value, msf_tmp::CorrectionStateLengthForType>::value;

	std::cout<<"idxstartcorr_p_ "<<idxstartcorr_p_<<std::endl;
	std::cout<<"idxstartcorr_v_ "<<idxstartcorr_v_<<std::endl;
	std::cout<<"idxstartcorr_q_ "<<idxstartcorr_q_<<std::endl;
	std::cout<<"idxstartcorr_b_w_ "<<idxstartcorr_b_w_<<std::endl;
	std::cout<<"idxstartcorr_b_a_ "<<idxstartcorr_b_a_<<std::endl;

	Eigen::Matrix<double, 3, 1> v1;
	v1<<1,2,4;

	Eigen::Matrix<double, 3, 3> m;

	m = (v1.cwiseProduct(v1)).asDiagonal();
	std::cout<<(v1.cwiseProduct(v1))<<std::endl<<std::endl;
	std::cout<<m<<std::endl;

	Eigen::Matrix<double, 5, 5> n;

	std::cout<<n<<std::endl;

	Eigen::Matrix<double, 3,3> other;
//n(2,2)=other(0,0);
	n.block<2,2>(2,2) = other.block<2,2>(0,0);


	std::cout<<n<<std::endl;

}

