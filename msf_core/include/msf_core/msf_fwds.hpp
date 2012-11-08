/*
 *  Created on: Nov 7, 2012
 *      Author: slynen
 */


#ifndef MSF_FWD_HPP_
#define MSF_FWD_HPP_

namespace msf_core{

//state types used to identify propagated/core/auxiliary states
enum{
	CoreStateWithPropagation,
	CoreStateWithoutPropagation,
	AuxiliaryState
};

//forwards
//state variable
template<typename type_T, int name_T, int STATETYPE = AuxiliaryState>
struct StateVar_T;

//the state
template<typename stateVector_T>
struct GenericState_T;

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


//abstract class defining user configurable calculations for the msf_core
class UserDefinedCalculationBase;

}
#endif /* MSF_FWD_HPP_ */
