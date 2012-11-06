/*
 * teststaticstatelist.cc
 *
 *  Created on: Nov 5, 2012
 *      Author: slynen
 */

#define FUSION_MAX_VECTOR_SIZE 20 //maximum number of statevariables (can be set to a larger value)

#include <Eigen/Dense>
#include <sstream>
#include <iostream>
#include <boost/preprocessor/punctuation/comma.hpp>
#include <boost/fusion/container/vector.hpp>
#include <boost/fusion/include/vector.hpp>
#include <boost/fusion/container/vector/vector_fwd.hpp>
#include <boost/fusion/include/vector_fwd.hpp>
#include <boost/fusion/sequence/intrinsic/size.hpp>
#include <boost/fusion/include/size.hpp>
#include <boost/fusion/include/at_c.hpp>
#include <boost/fusion/sequence/intrinsic/at.hpp>
#include <boost/fusion/include/at.hpp>
#include <boost/static_assert.hpp>
#include <boost/fusion/algorithm/iteration/for_each.hpp>
#include <boost/fusion/include/for_each.hpp>
#include <boost/fusion/algorithm/iteration/accumulate.hpp>
#include <boost/fusion/include/accumulate.hpp>
#include <boost/fusion/algorithm/query/count_if.hpp>
#include <boost/fusion/include/count_if.hpp>
#include <boost/fusion/include/equal_to.hpp>
#include <boost/fusion/include/begin.hpp>
#include <boost/fusion/include/end.hpp>
#include <boost/fusion/include/next.hpp>
#include <boost/mpl/bool.hpp>
#include <boost/fusion/include/at_key.hpp>

#include <msf_core/msf_tmp.hpp>
#include <msf_core/msf_statedef.hpp>

int main(int argc, char** argv)
{
	//an instantiation of a state
	msf_core::EKFState somestate;

	//number of state variables
	std::cout<<"nstatevars: "<<msf_core::EKFState::nstatevars_<<std::endl;

	//number of states
	std::cout<<"nstates: "<<msf_core::EKFState::nstates_<<std::endl;

	//number of correction states
	std::cout<<"ncorrectionstates: "<<msf_core::EKFState::ncorrectionstates_<<std::endl;

	//apply correction to all states
	Eigen::Matrix<double, msf_core::EKFState::ncorrectionstates_, 1> correction;
	std::cout<<"passing random correction vector"<<std::endl;
	correction.setRandom();
	somestate.correct(correction);
	//	std::cout<<"passing random correction vector"<<std::endl;
	//	correction.setRandom();
	//	somestate.correct(correction);




}
