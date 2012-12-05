/*

Copyright (c) 2012, Simon Lynen, ASL, ETH Zurich, Switzerland
You can contact the author at <slynen at ethz dot org>

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

#ifndef SENSORMANAGER_H
#define SENSORMANAGER_H

#include <msf_core/msf_statedef.hpp>
#include <msf_core/msf_state.hpp>
#include <Eigen/Dense>

namespace msf_core{

class SensorHandler;
class MSF_Core;

class MSF_SensorManager:public StateVisitor
{
protected:
	typedef std::vector<boost::shared_ptr<SensorHandler> > Handlers;
	Handlers handlers;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	boost::shared_ptr<MSF_Core> msf_core_;

	MSF_SensorManager();

	virtual ~MSF_SensorManager(){

	};

	void addHandler(boost::shared_ptr<SensorHandler> handler)
	{
		handlers.push_back(handler);
	}

	//init functions for the EKF
	virtual void init(double scale) = 0;

	//this method will be called for the user to set the initial state
	virtual void initState(msf_core::EKFState& state) = 0;

	//this method will be called for the user to set the Q block entries for Auxiliary states
	//only changes to blocks in Q belonging to the auxiliary states are allowed / evaluated
	virtual void calculateQAuxiliaryStates(msf_core::EKFState& state, double dt){};

	//this method will be called for the user to set the initial P matrix
	virtual void setP(Eigen::Matrix<double, msf_core::EKFState::nErrorStatesAtCompileTime, msf_core::EKFState::nErrorStatesAtCompileTime>& P) = 0;

	//this method will be called for the user to have the possibility to augment the correction vector
	virtual void augmentCorrectionVector(Eigen::Matrix<double, EKFState::nErrorStatesAtCompileTime,1>& correction){};

	virtual void sanityCheckCorrection(msf_core::EKFState& delaystate, const msf_core::EKFState& buffstate,
			Eigen::Matrix<double, EKFState::nErrorStatesAtCompileTime,1>& correction){};

	//provide a getter for these parameters
	virtual bool getParam_fixed_bias() = 0;
	virtual double getParam_delay() = 0;
	virtual double getParam_noise_acc() = 0;
	virtual double getParam_noise_accbias() = 0;
	virtual double getParam_noise_gyr() = 0;
	virtual double getParam_noise_gyrbias() = 0;
	virtual double getParam_fuzzythres() = 0;

};

class SensorHandler
{
protected:
	MSF_SensorManager* measurements;
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	SensorHandler(MSF_SensorManager* meas):measurements(meas){}
	virtual ~SensorHandler() {}
};

}; // end msf_core

#include <msf_core/implementation/msf_sensormanager.hpp>

#endif /* SENSORMANAGER_H */
