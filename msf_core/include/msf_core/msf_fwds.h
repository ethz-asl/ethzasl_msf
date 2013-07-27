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
#ifndef MSF_FWD_HPP_
#define MSF_FWD_HPP_

#include <msf_core/msf_types.hpp>
#include <msf_core/msf_macros.h>

namespace msf_core {

//state types used to identify propagated/core/auxiliary states
enum {
  CoreStateWithPropagation,
  CoreStateWithoutPropagation,
  Auxiliary,
  AuxiliaryNonTemporalDrifting
};

enum {  //set power 2 flags here
  none = 0x0,
  correctionMultiplicative = 0x1
};

//forwards

//state variable
template<typename type_T, int name_T, int STATETYPE = Auxiliary, int OPTIONS =
    none>
struct StateVar_T;

//the state
template<typename StateVector_T, typename StateDefinition_T>
struct GenericState_T;

template<typename EKFState_T>
class MSF_Core;

template<typename T> struct echoStateVarType;
template<typename T> struct CorrectionStateLengthForType;
template<typename T> struct StateLengthForType;
template<typename T> struct getEnumStateName;

template<typename Sequence, template<typename > class Counter, typename First,
    typename Last, bool T>
struct countStatesLinear;

template<typename Sequence, typename First, typename Last, int CurrentIdx,
    bool T>
struct CheckStateIndexing;

template<typename TypeList, int INDEX> struct getEnumStateType;

template<typename Sequence, typename StateVarT,
    template<typename > class OffsetCalculator, typename First, typename Last,
    bool TypeFound, int CurrentVal, bool EndOfList>
struct ComputeStartIndex;

template<typename Sequence> struct CheckCorrectIndexing;
template<typename Sequence, template<typename > class Counter>
struct CountStates;

template<typename Sequence, typename StateVarT,
    template<typename > class Counter>
struct getStartIndex;

struct resetState;

template<typename stateT> struct copyNonPropagationStates;
template<typename stateList_T> struct copyQBlocksFromAuxiliaryStatesToQ;
template<typename T, typename stateList_T> struct correctState;
template<typename T, typename stateList_T> struct StatetoDoubleArray;

class MeasurementHandler;
template<typename EKFState_T> class MSF_InitMeasurement;
template<typename EKFState_T> class MSF_MeasurementBase;
template<typename EKFState_T> class MSF_InvalidMeasurement;
template<typename EKFState_T> class MSF_SensorHandler;
template<typename EKFState_T> class MSF_SensorManager;

}
#endif /* MSF_FWD_HPP_ */
