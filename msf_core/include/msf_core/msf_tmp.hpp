/*
 *  Created on: Nov 7, 2012
 *      Author: slynen
 */

#ifndef MSF_TMP_HPP_
#define MSF_TMP_HPP_


#include <msf_core/msf_fwds.hpp>
#include <msf_core/msf_typetraits.tpp>

#include <Eigen/Dense>
#include <sstream>
#include <iostream>

#define FUSION_MAX_VECTOR_SIZE 20 //maximum number of statevariables (can be set to a larger value)

#include <boost/lexical_cast.hpp>
#include <boost/static_assert.hpp>
#include <boost/preprocessor/punctuation/comma.hpp>
#include <boost/fusion/include/vector.hpp>
#include <boost/fusion/include/vector_fwd.hpp>
#include <boost/fusion/include/size.hpp>
#include <boost/fusion/include/at_c.hpp>
#include <boost/fusion/include/at.hpp>
#include <boost/fusion/include/for_each.hpp>
#include <boost/fusion/include/equal_to.hpp>
#include <boost/fusion/include/begin.hpp>
#include <boost/fusion/include/end.hpp>
#include <boost/fusion/include/next.hpp>
#include <boost/mpl/bool.hpp>
#include <boost/fusion/include/at_key.hpp>

//this namespace contains some metaprogramming tools
namespace msf_tmp{

//output a compile time constant
template<size_t size>
struct overflow{ operator char() { return size + 256; } }; //always overflow
template<int VALUE>
void echoCompileTimeConstant()
{
	char(overflow<VALUE>());
	return;
}

//runtime output of stateVariable types
template<typename T> struct echoStateVarType;
template<int NAME, int N, int STATETYPE> struct echoStateVarType<const msf_core::StateVar_T<Eigen::Matrix<double, N, 1>, NAME, STATETYPE>&>{
	static std::string value(){
		return "const ref Eigen::Matrix<double, "+boost::lexical_cast<std::string>(N)+", 1>";
	}
};
template<int NAME, int STATETYPE> struct echoStateVarType<const msf_core::StateVar_T<Eigen::Quaterniond, NAME, STATETYPE>&>{
	static std::string value(){
		return "const ref Eigen::Quaterniond";
	}
};
template<int NAME, int N, int STATETYPE> struct echoStateVarType<msf_core::StateVar_T<Eigen::Matrix<double, N, 1>, NAME, STATETYPE> >{
	static std::string value(){
		return "Eigen::Matrix<double, "+boost::lexical_cast<std::string>(N)+", 1>";
	}
};
template<int NAME, int STATETYPE> struct echoStateVarType<msf_core::StateVar_T<Eigen::Quaterniond, NAME, STATETYPE> >{
	static std::string value(){
		return "Eigen::Quaterniond";
	}
};


namespace{ //for internal use only
//the number of entries in the correction vector for a given state var
template<typename T>
struct CorrectionStateLengthForType;
template<int NAME, int N, int STATETYPE> struct CorrectionStateLengthForType<const msf_core::StateVar_T<Eigen::Matrix<double, N, 1>, NAME, STATETYPE>& >{
	enum{value = N};
};
template<int NAME, int STATETYPE> struct CorrectionStateLengthForType<const msf_core::StateVar_T<Eigen::Quaterniond, NAME, STATETYPE>& >{
	enum{value = 3};
};
template<> struct CorrectionStateLengthForType<const mpl_::void_&>{
	enum{value = 0};
};

//the number of entries in the state for a given state var
template<typename T>
struct StateLengthForType;
template<int NAME, int N, int STATETYPE> struct StateLengthForType<const msf_core::StateVar_T<Eigen::Matrix<double, N, 1>, NAME, STATETYPE>& >{
	enum{value = N};
};
template<int NAME, int STATETYPE> struct StateLengthForType<const msf_core::StateVar_T<Eigen::Quaterniond, NAME, STATETYPE>& >{
	enum{value = 4};
};
template<> struct StateLengthForType<const mpl_::void_&>{
	enum{value = 0};
};


//the number of entries in the state for a given state var if it is core state
template<typename T>
struct CoreStateLengthForType;
template<int NAME, int N, int STATETYPE> struct CoreStateLengthForType<const msf_core::StateVar_T<Eigen::Matrix<double, N, 1>, NAME, STATETYPE>& >{
	enum{value = 0}; //not a core state, so length is zero
};
template<int NAME, int STATETYPE> struct CoreStateLengthForType<const msf_core::StateVar_T<Eigen::Quaterniond, NAME, STATETYPE>& >{
	enum{value = 0}; //not a core state, so length is zero
};
template<int NAME, int N> struct CoreStateLengthForType<const msf_core::StateVar_T<Eigen::Matrix<double, N, 1>, NAME, msf_core::CoreStateWithoutPropagation>& >{
	enum{value = N};
};
template<int NAME> struct CoreStateLengthForType<const msf_core::StateVar_T<Eigen::Quaterniond, NAME, msf_core::CoreStateWithoutPropagation>& >{
	enum{value = 4};
};
template<int NAME, int N> struct CoreStateLengthForType<const msf_core::StateVar_T<Eigen::Matrix<double, N, 1>, NAME, msf_core::CoreStateWithPropagation>& >{
	enum{value = N};
};
template<int NAME> struct CoreStateLengthForType<const msf_core::StateVar_T<Eigen::Quaterniond, NAME, msf_core::CoreStateWithPropagation>& >{
	enum{value = 4};
};
template<> struct CoreStateLengthForType<const mpl_::void_&>{
	enum{value = 0};
};

//return the number a state has in the enum
template<typename T>
struct getEnumStateName;
template<typename U,int NAME, int STATETYPE> struct getEnumStateName<const msf_core::StateVar_T<U, NAME, STATETYPE>& >{
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
		countStatesLinear<Sequence, Counter, Next, Last,
		SameType<typename msf_tmp::StripConstReference<First>::value,
		typename msf_tmp::StripConstReference<Last>::value>::value>::value
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

		indexingerrors = boost::mpl::if_c<idxInEnum == idxInState,
		boost::mpl::int_<0>, boost::mpl::int_<1> >::type::value + //error here
		CheckStateIndexing<Sequence, Next, Last, CurrentIdx + 1, 		//and errors of other states
		SameType<typename msf_tmp::StripConstReference<First>::value,
		typename msf_tmp::StripConstReference<Last>::value>::value>::indexingerrors

	};
private:
	//Error the ordering in the enum defining the names of the states is not the same ordering as in the type vector for the states
	BOOST_STATIC_ASSERT_MSG(indexingerrors==0, "Error the ordering in the enum defining the names _ "
			"of the states is not the same ordering as in the type vector for the states");
};
//}

template<typename TypeList, int INDEX>
struct getEnumStateType{
	typedef typename boost::fusion::result_of::at_c<TypeList, INDEX >::type value;
};

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

	enum{//the length of the current state plus the tail of the list
		value =  CurrentVal + ComputeStartIndex<Sequence, StateVarT, OffsetCalculator, Next, Last,
		SameType<typename msf_tmp::StripConstReference<currentType>::value,
		typename msf_tmp::StripConstReference<StateVarT>::value>::value,
		OffsetCalculator<currentType>::value, SameType<typename msf_tmp::StripConstReference<First>::value,
		typename msf_tmp::StripConstReference<Last>::value>::value>::value
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
private:
	enum{
		startindex = 0, //must not change this
	};
public:
	enum{
		indexingerrors = CheckStateIndexing<Sequence, First, Last, startindex,
		SameType<typename msf_tmp::StripConstReference<First>::value,
		typename msf_tmp::StripConstReference<Last>::value>::value>::indexingerrors
	};
};

//returns the number of doubles in the state/correction state
//depending on the counter type supplied
template<typename Sequence,  template<typename> class Counter>
struct CountStates{
	typedef typename boost::fusion::result_of::begin<Sequence const>::type First;
	typedef typename boost::fusion::result_of::end<Sequence const>::type Last;
	enum{
		value = countStatesLinear<Sequence, Counter, First, Last,
		SameType<typename msf_tmp::StripConstReference<First>::value,
		typename msf_tmp::StripConstReference<Last>::value>::value>::value
		+ CheckCorrectIndexing<Sequence>::indexingerrors //will be zero, if no indexing errors, otherwise fails compilation
	};
};

//compute start indices in the correction/state vector of a given type
template<typename Sequence, typename StateVarT, template<typename> class Counter>
struct getStartIndex{
	typedef typename boost::fusion::result_of::begin<Sequence const>::type First;
	typedef typename boost::fusion::result_of::end<Sequence const>::type Last;
	typedef typename  boost::fusion::result_of::deref<First>::type currentType;
	enum{
		value = ComputeStartIndex<Sequence, StateVarT, Counter, First, Last,
		SameType<typename msf_tmp::StripConstReference<StateVarT>::value,
		typename msf_tmp::StripConstReference<currentType>::value>::value, 0, SameType<First, Last>::value>::value
		+ CheckCorrectIndexing<Sequence>::indexingerrors //will be zero, if no indexing errors, otherwise fails compilation
	};
};

//reset the EKF state
struct resetState
{
	template<int NAME, int N, int STATETYPE>
	void operator()(msf_core::StateVar_T<Eigen::Matrix<double, N, 1>, NAME, STATETYPE>& t) const {
		typedef msf_core::StateVar_T<Eigen::Matrix<double, N, 1>, NAME, STATETYPE> var_T;
		t.state_.setZero();
	}
	template<int NAME, int STATETYPE>
	void operator()(msf_core::StateVar_T<Eigen::Quaterniond, NAME, STATETYPE>& t) const {
		typedef msf_core::StateVar_T<Eigen::Quaterniond, NAME, STATETYPE> var_T;
		t.state_.setIdentity();
	}
};

//copy states from previous to current states, for which there is no propagation
template<typename stateT>
struct copyNonPropagationStates
{
	copyNonPropagationStates(stateT& oldstate):oldstate_(oldstate){	}
	template<typename T, int NAME>
	void operator()(msf_core::StateVar_T<T, NAME, msf_core::CoreStateWithPropagation>& t) const {
		//nothing to do for the states, which have propagation
	}
	template<typename T, int NAME, int STATETYPE>
	void operator()(msf_core::StateVar_T<T, NAME, STATETYPE>& t) const {
		t = oldstate_.template get<NAME>(); //copy value from old state to new state var
		BOOST_STATIC_ASSERT_MSG(STATETYPE !=  msf_core::CoreStateWithPropagation,
								"copyNonPropagationStates was instantiated for a core state. This is an error.");
	}
private:
	stateT& oldstate_;
};

//copy the user calculated values in the Q-blocks to the main Q matrix
template<typename stateList_T>
struct copyQBlocksFromAuxiliaryStatesToQ
{
	enum{
		nErrorStatesAtCompileTime = msf_tmp::CountStates<stateList_T, msf_tmp::CorrectionStateLengthForType>::value //n correction states
	};
	typedef Eigen::Matrix<double, nErrorStatesAtCompileTime, nErrorStatesAtCompileTime> Q_T;
	copyQBlocksFromAuxiliaryStatesToQ(Q_T& Q):Q_(Q){	}
	template<typename T, int NAME>
	void operator()(msf_core::StateVar_T<T, NAME, msf_core::AuxiliaryState>& t) const {
		typedef msf_core::StateVar_T<T, NAME, msf_core::AuxiliaryState> var_T;
		enum{
			startIdxInCorrection = msf_tmp::getStartIndex<stateList_T, var_T, msf_tmp::CorrectionStateLengthForType>::value,
			sizeInCorrection = var_T::sizeInCorrection_
		};
		BOOST_STATIC_ASSERT_MSG(static_cast<int>(var_T::statetype) != static_cast<int>(msf_core::CoreStateWithPropagation) &&
				static_cast<int>(var_T::statetype) != static_cast<int>(msf_core::CoreStateWithoutPropagation),
						"copyQBlocksFromAuxiliaryStatesToQ was instantiated for a core state. This is an error.");
		BOOST_STATIC_ASSERT_MSG(static_cast<int>(sizeInCorrection) == static_cast<int>(var_T::Q_T::ColsAtCompileTime) &&
				static_cast<int>(sizeInCorrection) == static_cast<int>(var_T::Q_T::RowsAtCompileTime),
				"copyQBlocksFromAuxiliaryStatesToQ size of Matrix Q stored with the stateVar,"
				"is not the same as the reported dimension in the correction vector");

		Q_.template block<sizeInCorrection, sizeInCorrection>(startIdxInCorrection, startIdxInCorrection) = t.Q_;
	}
	template<typename T, int NAME, int STATETYPE>
	void operator()(msf_core::StateVar_T<T, NAME, STATETYPE>& t) const {
		BOOST_STATIC_ASSERT_MSG(STATETYPE == msf_core::CoreStateWithPropagation || STATETYPE == msf_core::CoreStateWithoutPropagation,
								"copyQBlocksFromAuxiliaryStatesToQ was instantiated for something else than an auxiliary state, "
								"but the handed over type is not a core state. This is an error.");
		//nothing to do for the states, which have propagation, because Q calculation is done in msf_core
	}
private:
	Q_T& Q_;
};

//apply EKF corrections depending on the stateVar type
template<typename T, typename stateList_T>
struct correctState
{
	correctState(T& correction):data_(correction){	}
	template<int NAME, int N, int STATETYPE>
	void operator()(msf_core::StateVar_T<Eigen::Matrix<double, N, 1>, NAME, STATETYPE>& t) const {
		typedef msf_core::StateVar_T<Eigen::Matrix<double, N, 1>, NAME, STATETYPE> var_T;
		std::cout<<"called correction for state "<<NAME<<" of type "<<msf_tmp::echoStateVarType<var_T>::value()<<std::endl;
		//get index of the data in the correction vector
		enum{
			startIdxInCorrection = msf_tmp::getStartIndex<stateList_T, var_T, msf_tmp::CorrectionStateLengthForType>::value,
			idxstartstate = msf_tmp::getStartIndex<stateList_T, var_T, msf_tmp::StateLengthForType>::value
		};

		//TODO implement

		std::cout<<"startindex in correction: "<<startIdxInCorrection<< " size in correction: "<<var_T::sizeInCorrection_<<std::endl;
		std::cout<<"startindex in state: "<<idxstartstate<<" size in state: "<<var_T::sizeInState_<<std::endl;

	}
	template<int NAME, int STATETYPE>
	void operator()(msf_core::StateVar_T<Eigen::Quaterniond, NAME, STATETYPE>& t) const {
		typedef msf_core::StateVar_T<Eigen::Quaterniond, NAME, STATETYPE> var_T;
		std::cout<<"called correction for state "<<NAME<<" of type "<<msf_tmp::echoStateVarType<var_T>::value()<<std::endl;
		//get index of the data in the correction vector
		enum{
			startIdxInCorrection = msf_tmp::getStartIndex<stateList_T, var_T, msf_tmp::CorrectionStateLengthForType>::value,
			idxstartstate = msf_tmp::getStartIndex<stateList_T, var_T, msf_tmp::StateLengthForType>::value
		};

		//TODO implement

		std::cout<<"startindex in correction: "<<startIdxInCorrection<< " size in correction: "<<var_T::sizeInCorrection_<<std::endl;
		std::cout<<"startindex in state: "<<idxstartstate<<" size in state: "<<var_T::sizeInState_<<std::endl;
	}
private:
	T& data_;
};

//copies the values of the single state vars to the double array provided
template<typename T, typename stateList_T>
struct CoreStatetoDoubleArray
{
	CoreStatetoDoubleArray(T& statearray):data_(statearray){}
	template<int NAME, int N, int STATETYPE>
	void operator()(msf_core::StateVar_T<Eigen::Matrix<double, N, 1>, NAME, STATETYPE>& t) const {
		typedef msf_core::StateVar_T<Eigen::Matrix<double, N, 1>, NAME, STATETYPE> var_T;
		enum{
			startIdxInState = msf_tmp::getStartIndex<stateList_T, var_T, msf_tmp::StateLengthForType>::value //index of the data in the state vector
		};
		BOOST_STATIC_ASSERT_MSG(var_T::statetype == msf_core::CoreStateWithPropagation || var_T::statetype == msf_core::CoreStateWithoutPropagation,
				"CoreStatetoDoubleArray: A new statetype was defined, but I don't know whether this should be written to the double array for core states");
		for(int i = 0;i<var_T::sizeInState_;++i){
			data_[startIdxInState + i] = t.state_[i];
		}
	}
	template<int NAME, int STATETYPE>
	void operator()(msf_core::StateVar_T<Eigen::Quaterniond, NAME, STATETYPE>& t) const {
		typedef msf_core::StateVar_T<Eigen::Quaterniond, NAME, STATETYPE> var_T;
		enum{
			startIdxInState = msf_tmp::getStartIndex<stateList_T, var_T, msf_tmp::StateLengthForType>::value //index of the data in the state vector
		};
		BOOST_STATIC_ASSERT_MSG(var_T::statetype == msf_core::CoreStateWithPropagation || var_T::statetype == msf_core::CoreStateWithoutPropagation,
				"CoreStatetoDoubleArray: A new statetype was defined, but I don't know whether this should be written to the double array for core states");
		//copy quaternion values
		data_[startIdxInState + 0] = t.state_.w();
		data_[startIdxInState + 1] = t.state_.x();
		data_[startIdxInState + 2] = t.state_.y();
		data_[startIdxInState + 3] = t.state_.z();
	}
	template<int NAME, int N>
	void operator()(msf_core::StateVar_T<Eigen::Matrix<double, N, 1>, NAME, msf_core::AuxiliaryState>& t) const {
		//don't copy aux states
	}
	template<int NAME>
	void operator()(msf_core::StateVar_T<Eigen::Quaterniond, NAME, msf_core::AuxiliaryState>& t) const {
		//don't copy aux states
	}
private:
	T& data_;
};

//copies the values of the single state vars to the double array provided
template<typename T, typename stateList_T>
struct FullStatetoDoubleArray
{
	FullStatetoDoubleArray(T& statearray):data_(statearray){}
	template<int NAME, int N, int STATETYPE>
	void operator()(msf_core::StateVar_T<Eigen::Matrix<double, N, 1>, NAME, STATETYPE>& t) const {
		typedef msf_core::StateVar_T<Eigen::Matrix<double, N, 1>, NAME, STATETYPE> var_T;
		enum{
			startIdxInState = msf_tmp::getStartIndex<stateList_T, var_T, msf_tmp::StateLengthForType>::value //index of the data in the state vector
		};
		for(int i = 0;i<var_T::sizeInState_;++i){
			data_[startIdxInState + i] = t.state_[i];
		}
	}
	template<int NAME, int STATETYPE>
	void operator()(msf_core::StateVar_T<Eigen::Quaterniond, NAME, STATETYPE>& t) const {
		typedef msf_core::StateVar_T<Eigen::Quaterniond, NAME, STATETYPE> var_T;
		enum{
			startIdxInState = msf_tmp::getStartIndex<stateList_T, var_T, msf_tmp::StateLengthForType>::value //index of the data in the state vector
		};
		//copy quaternion values
		data_[startIdxInState + 0] = t.state_.w();
		data_[startIdxInState + 1] = t.state_.x();
		data_[startIdxInState + 2] = t.state_.y();
		data_[startIdxInState + 3] = t.state_.z();
	}
private:
	T& data_;
};
}

#endif /* MSF_TMP_HPP_ */
