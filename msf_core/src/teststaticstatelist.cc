/*
 * teststaticstatelist.cc
 *
 *  Created on: Nov 5, 2012
 *      Author: slynen
 */

#include <msf_core/msf_core.hpp>
#include <gtest/gtest.h>

#define WITHTESTS 0

#if (WITHTESTS == 1)
//test calculated sizes
TEST(CompileTimeComputation, stateSizeCalculation) {
	enum{
		a,
		b,
		c,
		d
	};
	const static int vectorlength1 = 4;
	const static int vectorlength2 = 10;

	//setup some state type
	typedef boost::fusion::vector<
			StateVar_T<Eigen::Matrix<double, vectorlength1, 1>, a >,
			StateVar_T<Eigen::Quaterniond, b >,
			StateVar_T<Eigen::Matrix<double, 1, 1>, c >,
			StateVar_T<Eigen::Matrix<double, vectorlength2, 1>, d >
	> fullState_T;
	typedef EKFState_T<fullState_T> EKFState;

	EKFState somestate;
	ASSERT_EQ(somestate.get<0>().sizeInCorrection_, vectorlength1);
	ASSERT_EQ(somestate.get<1>().sizeInCorrection_, 3);
	ASSERT_EQ(somestate.get<2>().sizeInCorrection_, 1);
	ASSERT_EQ(somestate.get<3>().sizeInCorrection_, vectorlength2);

	ASSERT_EQ(somestate.get<0>().sizeInState_, vectorlength1);
	ASSERT_EQ(somestate.get<1>().sizeInState_, 4);
	ASSERT_EQ(somestate.get<2>().sizeInState_, 1);
	ASSERT_EQ(somestate.get<3>().sizeInState_, vectorlength2);
}

//test indices of statevars in state vectors
TEST(CompileTimeComputation, stateIndexCalculation) {
	enum{
		a,
		b,
		c,
		d
	};
	const static int vectorlength1 = 4;
	const static int vectorlength2 = 10;

	//setup some state type
	typedef boost::fusion::vector<
			StateVar_T<Eigen::Matrix<double, vectorlength1, 1>, a >,
			StateVar_T<Eigen::Quaterniond, b >,
			StateVar_T<Eigen::Matrix<double, 1, 1>, c >,
			StateVar_T<Eigen::Matrix<double, vectorlength2, 1>, d >
	> fullState_T;
	typedef EKFState_T<fullState_T> EKFState;

	EKFState somestate;
	static const int  idxstartcorr1 = msf_tmp::getStartIndex<fullState_T, StateVar_T<Eigen::Matrix<double, vectorlength1, 1>, a >, msf_tmp::CorrectionStateLengthForType>::value;
	static const int  idxstartstate1 = msf_tmp::getStartIndex<fullState_T, StateVar_T<Eigen::Matrix<double, vectorlength1, 1>, a >, msf_tmp::StateLengthForType>::value;
	ASSERT_EQ(idxstartcorr1, 0);
	ASSERT_EQ(idxstartstate1, 0);

	static const int  idxstartcorr2 = msf_tmp::getStartIndex<fullState_T, StateVar_T<Eigen::Quaterniond, b >, msf_tmp::CorrectionStateLengthForType>::value;
	static const int  idxstartstate2 = msf_tmp::getStartIndex<fullState_T, StateVar_T<Eigen::Quaterniond, b >, msf_tmp::StateLengthForType>::value;
	ASSERT_EQ(idxstartcorr2, vectorlength1);
	ASSERT_EQ(idxstartstate2, vectorlength1);

	static const int  idxstartcorr3 = msf_tmp::getStartIndex<fullState_T, StateVar_T<Eigen::Matrix<double, 1, 1>, c >, msf_tmp::CorrectionStateLengthForType>::value;
	static const int  idxstartstate3 = msf_tmp::getStartIndex<fullState_T, StateVar_T<Eigen::Matrix<double, 1, 1>, c >, msf_tmp::StateLengthForType>::value;
	ASSERT_EQ(idxstartcorr3, vectorlength1 + 3);
	ASSERT_EQ(idxstartstate3, vectorlength1 + 4);

	static const int  idxstartcorr4 = msf_tmp::getStartIndex<fullState_T, StateVar_T<Eigen::Matrix<double, vectorlength2, 1>, d >, msf_tmp::CorrectionStateLengthForType>::value;
	static const int  idxstartstate4 = msf_tmp::getStartIndex<fullState_T, StateVar_T<Eigen::Matrix<double, vectorlength2, 1>, d >, msf_tmp::StateLengthForType>::value;
	ASSERT_EQ(idxstartcorr4, vectorlength1 + 3 + 1);
	ASSERT_EQ(idxstartstate4, vectorlength1 + 4 + 1);

}

// Tests compile time computed values for the state
TEST(CompileTimeComputation, stateLengthCalculation) {
	enum{
		a,
		b,
		c,
		d
	};
	const static int vectorlength1 = 3;
	const static int vectorlength2 = 20;

	//setup some state type
	typedef boost::fusion::vector<
			StateVar_T<Eigen::Matrix<double, vectorlength1, 1>, a >,
			StateVar_T<Eigen::Quaterniond, b >,
			StateVar_T<Eigen::Matrix<double, 1, 1>, c >,
			StateVar_T<Eigen::Matrix<double, vectorlength2, 1>, d >
	> fullState_T;
	typedef EKFState_T<fullState_T> EKFState;

	ASSERT_EQ(EKFState::nstatevars_, 4);
	ASSERT_EQ(EKFState::nstates_, vectorlength1 + vectorlength2 + 4 + 1);
	ASSERT_EQ(EKFState::nerrorstates_, vectorlength1 + vectorlength2 + 3 +1);
}
#endif


int main(int argc, char** argv)
{

	//an instantiation of a state
	msf_core::EKFState somestate;

	std::cout<<"name "<<somestate.get<0>().name_<<std::endl;

	//number of state variables
	std::cout<<"nstatevars: "<<msf_core::EKFState::nstatevars_<<std::endl;

	//number of states
	std::cout<<"nstates: "<<msf_core::EKFState::nstates_<<std::endl;

	//number of correction states
	std::cout<<"nerrortates: "<<msf_core::EKFState::nerrorstates_<<std::endl;

	//apply correction to all states
	Eigen::Matrix<double, msf_core::EKFState::nerrorstates_, 1> correction;
	std::cout<<"passing random correction vector"<<std::endl;
	correction.setRandom();
	somestate.correct(correction);

#if (WITHTESTS == 1)
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
#endif
}
