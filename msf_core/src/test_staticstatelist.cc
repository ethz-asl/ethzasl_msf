/*
 * teststaticstatelist.cc
 *
 *  Created on: Nov 5, 2012
 *      Author: slynen
 */

#include <msf_core/msf_core.hpp>
#include <gtest/gtest.h>

#define WITHTESTS 1

#if (WITHTESTS == 1)
//test calculated sizes
TEST(CompileTimeComputation, stateSizeCalculation) {
	using namespace msf_core;
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
	typedef GenericState_T<fullState_T> EKFState;

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
	using namespace msf_core;
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
	typedef GenericState_T<fullState_T> EKFState;

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
	using namespace msf_core;
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
	typedef GenericState_T<fullState_T> EKFState;

	ASSERT_EQ(EKFState::nStateVarsAtCompileTime, 4);
	ASSERT_EQ(EKFState::nStatesAtCompileTime, vectorlength1 + vectorlength2 + 4 + 1);
	ASSERT_EQ(EKFState::nErrorStatesAtCompileTime, vectorlength1 + vectorlength2 + 3 +1);
}


TEST(RuntimeTimeComputation, copyForNonPropagationStates) {

	enum{ //must not manually set the enum values!
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

	//setup core state, then auxiliary state
	typedef boost::fusion::vector<
			// states varying during propagation - must not change the ordering here for now, CalcQ has the ordering hardcoded
			msf_core::StateVar_T<Eigen::Matrix<double, 3, 1>, p_, true>,			///< position (IMU centered)          (0-2 / 0-2)
			msf_core::StateVar_T<Eigen::Matrix<double, 3, 1>, v_, true>,			///< velocity                         (3- 5 / 3- 5)
			msf_core::StateVar_T<Eigen::Quaternion<double>, q_, true>,				///< attitude                         (6- 9 / 6- 8)
			msf_core::StateVar_T<Eigen::Matrix<double, 3, 1>, b_w_, true>,			///< gyro biases                      (10-12 / 9-11)
			msf_core::StateVar_T<Eigen::Matrix<double, 3, 1>, b_a_, true>,			///< acceleration biases              (13-15 / 12-14)

			// states not varying during propagation
			msf_core::StateVar_T<Eigen::Matrix<double, 1, 1>, L_>,			///< visual scale                     (16 / 15)
			msf_core::StateVar_T<Eigen::Quaternion<double>, q_wv_>,			///< vision-world attitude drift      (17-20 / 16-18)
			msf_core::StateVar_T<Eigen::Quaternion<double>, q_ci_>,			///< camera-imu attitude calibration  (21-24 / 19-21)
			msf_core::StateVar_T<Eigen::Matrix<double, 3, 1>, p_ci_>			///< camera-imu position calibration  (25-27 / 22-24)
	> fullState_T;

	typedef msf_core::GenericState_T<fullState_T> EKFState;

	EKFState first_state;
	EKFState second_state;

	first_state.reset();
	second_state.reset();


	first_state.get<msf_core::p_>().state_.setRandom();

	first_state.get<msf_core::q_>().state_.w() = 0.4;
	first_state.get<msf_core::q_>().state_.x() = 0.1;
	first_state.get<msf_core::q_>().state_.y() = 0.2;
	first_state.get<msf_core::q_>().state_.z() = 0.7;
	first_state.get<msf_core::q_>().state_.normalize();

	boost::fusion::for_each(
			first_state.statevars_,
			msf_tmp::copyNonPropagationStates<EKFState>(second_state)
	);

	ASSERT_DOUBLE_EQ(first_state.get<msf_core::p_>().state_(0), second_state.get<msf_core::p_>().state_(0));
	ASSERT_DOUBLE_EQ(first_state.get<msf_core::p_>().state_(1), second_state.get<msf_core::p_>().state_(1));
	ASSERT_DOUBLE_EQ(first_state.get<msf_core::p_>().state_(2), second_state.get<msf_core::p_>().state_(2));

	ASSERT_DOUBLE_EQ(first_state.get<msf_core::q_>().state_.w(), second_state.get<msf_core::q_>().state_.w());
	ASSERT_DOUBLE_EQ(first_state.get<msf_core::q_>().state_.x(), second_state.get<msf_core::q_>().state_.x());
	ASSERT_DOUBLE_EQ(first_state.get<msf_core::q_>().state_.y(), second_state.get<msf_core::q_>().state_.y());
	ASSERT_DOUBLE_EQ(first_state.get<msf_core::q_>().state_.z(), second_state.get<msf_core::q_>().state_.z());
}

#endif


int main(int argc, char** argv)
{

	//an instantiation of a state
	msf_core::EKFState somestate;

	std::cout<<"name "<<somestate.get<0>().name_<<std::endl;

	//number of state variables
	std::cout<<"nstatevars: "<<msf_core::EKFState::nStatesAtCompileTime<<std::endl;

	//number of states
	std::cout<<"nstates: "<<msf_core::EKFState::nStatesAtCompileTime<<std::endl;

	//number of correction states
	std::cout<<"nerrortates: "<<msf_core::EKFState::nErrorStatesAtCompileTime<<std::endl;

	//apply correction to all states
	Eigen::Matrix<double, msf_core::EKFState::nErrorStatesAtCompileTime, 1> correction;
	std::cout<<"passing random correction vector"<<std::endl;
	correction.setRandom();
	somestate.correct(correction);

#if (WITHTESTS == 1)
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
#endif
}
