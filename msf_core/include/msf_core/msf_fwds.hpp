/*

Copyright (c) 2012, Simon Lynen, ASL, ETH Zurich, Switzerland
You can contact the author at <slynen at ethz dot ch>

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


#ifndef MSF_FWD_HPP_
#define MSF_FWD_HPP_

namespace msf_core{

#include <boost/shared_ptr.hpp>
#include <iostream>

//state types used to identify propagated/core/auxiliary states
enum{
	CoreStateWithPropagation,
	CoreStateWithoutPropagation,
	Auxiliary,
	AuxiliaryNonTemporalDrifting
};

//forwards
//state variable
template<typename type_T, int name_T, int STATETYPE = Auxiliary>
struct StateVar_T;

//the state
template<typename stateVector_T>
struct GenericState_T;

class MSF_Core;

template<typename T> struct echoStateVarType;
template<typename T>
struct CorrectionStateLengthForType;
template<typename T>
struct StateLengthForType;
template<typename T>
struct getEnumStateName;
template <typename Sequence,  template<typename> class Counter, typename First, typename Last, bool T>
struct countStatesLinear;
template <typename Sequence, typename First, typename Last, int CurrentIdx, bool T>
struct CheckStateIndexing;
template<typename TypeList, int INDEX>
struct getEnumStateType;

template <typename Sequence, typename StateVarT, template<typename> class OffsetCalculator,
typename First, typename Last, bool TypeFound, int CurrentVal, bool EndOfList>
struct ComputeStartIndex;
template<typename Sequence>
struct CheckCorrectIndexing;
template<typename Sequence,  template<typename> class Counter>
struct CountStates;
template<typename Sequence, typename StateVarT, template<typename> class Counter>
struct getStartIndex;
struct resetState;
template<typename stateT>
struct copyNonPropagationStates;
template<typename stateList_T>
struct copyQBlocksFromAuxiliaryStatesToQ;
template<typename T, typename stateList_T>
struct correctState;
template<typename T, typename stateList_T>
struct StatetoDoubleArray;

class MeasurementHandler;
class MSF_InitMeasurement;
class MSF_MeasurementBase;
class MSF_InvalidMeasurement;

}
#endif /* MSF_FWD_HPP_ */
