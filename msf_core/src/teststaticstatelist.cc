/*
 * teststaticstatelist.cc
 *
 *  Created on: Nov 5, 2012
 *      Author: slynen
 */

#include <msf_core/msf_core.hpp>
#include <gtest/gtest.h>


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
