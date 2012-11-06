/*
 * msf_tmp.hpp
 *
 *  Created on: Nov 6, 2012
 *      Author: slynen
 */

#ifndef MSF_TMP_HPP_
#define MSF_TMP_HPP_

#include <msf_core/msf_fwd.hpp>
#include <boost/lexical_cast.hpp>

#include <Eigen/Dense>
#include <sstream>
#include <iostream>

#define FUSION_MAX_VECTOR_SIZE 20 //maximum number of statevariables (can be set to a larger value)

#include <boost/preprocessor/punctuation/comma.hpp>
#include <boost/fusion/include/vector.hpp>
#include <boost/fusion/include/vector_fwd.hpp>
#include <boost/fusion/include/size.hpp>
#include <boost/fusion/include/at_c.hpp>
#include <boost/fusion/include/at.hpp>
#include <boost/static_assert.hpp>
#include <boost/fusion/include/for_each.hpp>
#include <boost/fusion/include/equal_to.hpp>
#include <boost/fusion/include/begin.hpp>
#include <boost/fusion/include/end.hpp>
#include <boost/fusion/include/next.hpp>
#include <boost/mpl/bool.hpp>
#include <boost/fusion/include/at_key.hpp>

//this namespace contains some metaprogramming tools
namespace msf_tmp{

//runtime output of stateVariable types
template<typename T> struct echoStateVarType;
template<int NAME, int N> struct echoStateVarType<const StateVar_T<Eigen::Matrix<double, N, 1>, NAME>&>{
	static std::string value(){
		return "const ref Eigen::Matrix<double, "+boost::lexical_cast<std::string>(N)+", 1>";
	}
};
template<int NAME> struct echoStateVarType<const StateVar_T<Eigen::Quaterniond, NAME>&>{
	static std::string value(){
		return "const ref Eigen::Quaterniond";
	}
};
template<int NAME, int N> struct echoStateVarType<StateVar_T<Eigen::Matrix<double, N, 1>, NAME> >{
	static std::string value(){
		return "Eigen::Matrix<double, "+boost::lexical_cast<std::string>(N)+", 1>";
	}
};
template<int NAME> struct echoStateVarType<StateVar_T<Eigen::Quaterniond, NAME> >{
	static std::string value(){
		return "Eigen::Quaterniond";
	}
};


namespace{//hide these

template <typename T, typename U> struct SameType{enum { value = false };};
template <typename T> struct SameType<T,T>{enum { value = true };};

//the number of entries in the correction vector for a given state var
template<typename T>
struct CorrectionStateLengthForType;
template<int NAME, int N> struct CorrectionStateLengthForType<const StateVar_T<Eigen::Matrix<double, N, 1>, NAME>& >{
	enum{value = N};
};
template<int NAME> struct CorrectionStateLengthForType<const StateVar_T<Eigen::Quaterniond, NAME>& >{
	enum{value = 3};
};
template<> struct CorrectionStateLengthForType<const mpl_::void_&>{
	enum{value = 0};
};

//the number of entries in the state for a given state var
template<typename T>
struct StateLengthForType;
template<int NAME, int N> struct StateLengthForType<const StateVar_T<Eigen::Matrix<double, N, 1>, NAME>& >{
	enum{value = N};
};
template<int NAME> struct StateLengthForType<const StateVar_T<Eigen::Quaterniond, NAME>& >{
	enum{value = 4};
};
template<> struct StateLengthForType<const mpl_::void_&>{
	enum{value = 0};
};

//return the number a state has in the enum
template<typename T>
struct getEnumStateName;
template<typename U,int NAME> struct getEnumStateName<const StateVar_T<U, NAME>& >{
	enum{value = NAME};
};
template<> struct getEnumStateName<const mpl_::void_&>{
	enum{value = -1};
};

template <typename Sequence,  template<typename> class Counter, typename First, typename Last, bool T>
struct countStatesLinear;
template <typename Sequence,  template<typename> class Counter, typename First, typename Last>
struct countStatesLinear<Sequence, Counter, First, Last, true>{
	enum{//the end does not add entries
		value = 0
	};
};
template <typename Sequence, template<typename> class Counter, typename First, typename Last>
struct countStatesLinear<Sequence, Counter, First, Last, false>{
	typedef typename boost::fusion::result_of::next<First>::type Next;
	typedef typename  boost::fusion::result_of::deref<First>::type current_Type;
	enum{//the length of the current state plus the tail of the list
		value = Counter<current_Type>::value +
		countStatesLinear<Sequence, Counter, Next, Last, SameType<First, Last>::value>::value
	};
};


//helpers for CheckCorrectIndexing call{
template <typename Sequence, typename First, typename Last, int CurrentIdx, bool T>
struct CheckStateIndexing;
template <typename Sequence, typename First, typename Last, int CurrentIdx>
struct CheckStateIndexing<Sequence, First, Last, CurrentIdx, true>{
	enum{//no index no name, no indexing errors
		indexingerrors = 0
	};
};
template <typename Sequence, typename First, typename Last, int CurrentIdx>
struct CheckStateIndexing<Sequence, First, Last, CurrentIdx, false>{
	typedef typename boost::fusion::result_of::next<First>::type Next;
	typedef typename  boost::fusion::result_of::deref<First>::type current_Type;
	enum{//the length of the current state plus the tail of the list
		idxInEnum = boost::mpl::if_c<getEnumStateName<current_Type>::value != -1, //only evaluate if not at the end of the list
		boost::mpl::int_<getEnumStateName<current_Type>::value>, //if we're not at the end of the list, use the calculated enum index
		boost::mpl::int_<CurrentIdx> >::type::value, //else use the current index, so the assertion doesn't fail

		idxInState = CurrentIdx,

		indexingerrors = boost::mpl::if_c<idxInEnum == idxInState, boost::mpl::int_<0>, boost::mpl::int_<1> >::type::value + //error here and errors of other states
		CheckStateIndexing<Sequence, Next, Last, CurrentIdx + 1, SameType<First, Last>::value>::indexingerrors

	};
private:
	//Error the ordering in the enum defining the names of the states is not the same ordering as in the type vector for the states
	BOOST_STATIC_ASSERT_MSG(indexingerrors==0, "Error the ordering in the enum defining the names of the states is not the same ordering as in the type vector for the states");
};
//}


//helper for getStartIndex{
template <typename Sequence, typename StateVarT, template<typename> class OffsetCalculator,
typename First, typename Last, bool TypeFound, int CurrentVal, bool EndOfList>
struct ComputeStartIndex;
template <typename Sequence,  typename StateVarT, template<typename> class OffsetCalculator,
typename First, typename Last, int CurrentVal>
struct ComputeStartIndex<Sequence, StateVarT, OffsetCalculator, First, Last, false, CurrentVal, true>{
	enum{//return error code if end of list reached and type not found
		value = -1
	};
};
template <typename Sequence,  typename StateVarT, template<typename> class OffsetCalculator,
typename First, typename Last, bool TypeFound, int CurrentVal>
struct ComputeStartIndex<Sequence, StateVarT, OffsetCalculator, First, Last, TypeFound, CurrentVal, false>{
	enum{//we found the type, do not add additional offset
		value = 0
	};
};
template <typename Sequence, typename StateVarT, template<typename> class OffsetCalculator,
typename First, typename Last, int CurrentVal>
struct ComputeStartIndex<Sequence, StateVarT, OffsetCalculator, First, Last, false, CurrentVal, false>{
	typedef typename boost::fusion::result_of::next<First>::type Next;
	typedef typename  boost::fusion::result_of::deref<First>::type currentType;
	typedef const StateVarT& constRefLookupType;

	enum{//the length of the current state plus the tail of the list
		value =  CurrentVal + ComputeStartIndex<Sequence, StateVarT, OffsetCalculator, Next, Last, SameType<currentType, constRefLookupType>::value,
		OffsetCalculator<currentType>::value, SameType<First, Last>::value>::value
	};
};
//}
} //end anonymous namespace


//checks whether the ordering in the vector is the same as in the enum,
//which is something that strictly must not change
template<typename Sequence>
struct CheckCorrectIndexing{
	typedef typename boost::fusion::result_of::begin<Sequence const>::type First;
	typedef typename boost::fusion::result_of::end<Sequence const>::type Last;
	enum{
		startindex = 0, //must not change
		indexingerrors = CheckStateIndexing<Sequence, First, Last, startindex, SameType<First, Last>::value>::indexingerrors
	};
};

//returns the number of doubles in the state/correction state
//depending on the counter type supplied
template<typename Sequence,  template<typename> class Counter>
struct CountStates{
	typedef typename boost::fusion::result_of::begin<Sequence const>::type First;
	typedef typename boost::fusion::result_of::end<Sequence const>::type Last;
	enum{
		value = countStatesLinear<Sequence, Counter, First, Last, SameType<First, Last>::value>::value
		+ CheckCorrectIndexing<Sequence>::indexingerrors //will be zero, if no indexing errors, otherwise fails compilation
	};
};

//compute start indices in the correction/state vector of a given type
template<typename Sequence, typename StateVarT, template<typename> class Counter>
struct getStartIndex{
	typedef typename boost::fusion::result_of::begin<Sequence const>::type First;
	typedef typename boost::fusion::result_of::end<Sequence const>::type Last;
	typedef typename  boost::fusion::result_of::deref<First>::type currentType;
	typedef const StateVarT& constRefLookupType;
	enum{
		value = ComputeStartIndex<Sequence, StateVarT, Counter, First, Last, SameType<constRefLookupType, currentType>::value, 0, SameType<First, Last>::value>::value
		+ CheckCorrectIndexing<Sequence>::indexingerrors //will be zero, if no indexing errors, otherwise fails compilation
	};
};

//apply EKF corrections depending on the stateVar type
template<typename T, typename stateList_T>
struct applycorrection
{
	applycorrection(T& correction):data_(correction){
		std::cout<<"Got correction vector"<<correction.transpose()<<std::endl;
	}
	template<int NAME, int N>
	void operator()(StateVar_T<Eigen::Matrix<double, N, 1>, NAME>& t) const{
		typedef StateVar_T<Eigen::Matrix<double, N, 1>, NAME> var_T;
		std::cout<<"called correction for state "<<NAME<<" of type "<<msf_tmp::echoStateVarType<var_T>::value()<<std::endl;
		//get index of the data in the correction vector
		static const int  idxstartcorr = msf_tmp::getStartIndex<stateList_T, var_T, msf_tmp::CorrectionStateLengthForType>::value;
		static const int  idxstartstate = msf_tmp::getStartIndex<stateList_T, var_T, msf_tmp::StateLengthForType>::value;

		std::cout<<"startindex in correction: "<<idxstartcorr<< " size in correction: "<<var_T::sizeInCorrection_<<std::endl;
		std::cout<<"startindex in state: "<<idxstartstate<<" size in state: "<<var_T::sizeInState_<<std::endl;

	}
	template<int NAME>
	void operator()(StateVar_T<Eigen::Quaterniond, NAME>& t) const{
		typedef StateVar_T<Eigen::Quaterniond, NAME> var_T;
		std::cout<<"called correction for state "<<NAME<<" of type "<<msf_tmp::echoStateVarType<var_T>::value()<<std::endl;
		//get index of the data in the correction vector
		static const int idxstartcorr = msf_tmp::getStartIndex<stateList_T, var_T, msf_tmp::CorrectionStateLengthForType>::value;
		static const int idxstartstate = msf_tmp::getStartIndex<stateList_T, var_T, msf_tmp::StateLengthForType>::value;

		std::cout<<"startindex in correction: "<<idxstartcorr<< " size in correction: "<<var_T::sizeInCorrection_<<std::endl;
		std::cout<<"startindex in state: "<<idxstartstate<<" size in state: "<<var_T::sizeInState_<<std::endl;
	}
private:
	T& data_;
};

}


//a state variable with a name as specified in the state name enum
template<typename type_T, int name_T>
struct StateVar_T{
	typedef type_T state_T;
	typedef const StateVar_T<type_T, name_T>& constRef_T;
	typedef const StateVar_T<type_T, name_T>* constPtr_T;
	typedef StateVar_T<type_T, name_T>& Ref_T;
	typedef StateVar_T<type_T, name_T>* Ptr_T;
	enum{
		name_ = name_T,
		sizeInCorrection_ = msf_tmp::CorrectionStateLengthForType<const StateVar_T<type_T, name_T>&>::value,
		sizeInState_ = msf_tmp::StateLengthForType<const StateVar_T<type_T, name_T>&>::value
	};
	state_T state_;
};

template<typename stateVector_T>
struct EKFState_T{
	typedef stateVector_T state_T;
	stateVector_T statevars_;
	enum{
		nstatevars_ = boost::fusion::result_of::size<state_T>::type::value, //n all states
		ncorrectionstates_ = msf_tmp::CountStates<state_T, msf_tmp::CorrectionStateLengthForType>::value, //n correction states
		nstates_ = msf_tmp::CountStates<state_T, msf_tmp::StateLengthForType>::value //n correction states
	};

	//apply the correction vector to all state vars
	void correct(const Eigen::Matrix<double, ncorrectionstates_, 1>& correction) {
		boost::fusion::for_each(
				statevars_,
				msf_tmp::applycorrection<const Eigen::Matrix<double, ncorrectionstates_, 1>, state_T >(correction)
		);
	}

	//returns the state at position INDEX in the state list
	template<int INDEX>
	typename boost::fusion::result_of::at_c<state_T, INDEX >::type get(){
		return boost::fusion::at<boost::mpl::int_<INDEX> >(statevars_);
	}
};

#endif /* MSF_TMP_HPP_ */
