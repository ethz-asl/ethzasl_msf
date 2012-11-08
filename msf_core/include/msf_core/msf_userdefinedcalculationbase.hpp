/*
 * msf_userdefinedcalculationbase.hpp
 *
 *  Created on: Nov 8, 2012
 *      Author: slynen
 */

#ifndef MSF_USERDEFINEDCALCULATIONBASE_HPP_
#define MSF_USERDEFINEDCALCULATIONBASE_HPP_
#include <msf_core/msf_statedef.hpp>
#include <msf_core/msf_state.hpp>

namespace msf_core{
//abstract class defining user configurable calculations for the msf_core
class UserDefinedCalculationBase{
	enum{
		nErrorStatesAtCompileTime = EKFState::nErrorStatesAtCompileTime
	};

public:
	virtual ~UserDefinedCalculationBase(){}

	//the state is set to zero/identity, this method will be called to
	//give the user the possibility to change the reset values of some states
	virtual void resetState(msf_core::EKFState& state){}

	//this method will be called for the user to set the initial state
	virtual void initState(msf_core::EKFState& state) = 0;

	//this method will be called for the user to set the Q block entries for Auxiliary states
	//only changes to blocks in Q belonging to the auxiliary states are allowed / evaluated
	virtual void calculateQAuxiliaryStates(msf_core::EKFState& state, double dt){};

	//this method will be called for the user to set the initial P matrix
	virtual void setP(Eigen::Matrix<double, nErrorStatesAtCompileTime, nErrorStatesAtCompileTime>& P) = 0;

	//this method will be called for the user to have the possibility to augment the correction vector
	virtual void augmentCorrectionVector(Eigen::Matrix<double, nErrorStatesAtCompileTime,1>& correction){};

	//provide a getter for these parameters
	virtual bool getParam_fixed_bias() = 0;
	virtual double getParam_delay() = 0;
	virtual double getParam_noise_acc() = 0;
	virtual double getParam_noise_accbias() = 0;
	virtual double getParam_noise_gyr() = 0;
	virtual double getParam_noise_gyrbias() = 0;
};

}

#endif /* MSF_USERDEFINEDCALCULATIONBASE_HPP_ */
