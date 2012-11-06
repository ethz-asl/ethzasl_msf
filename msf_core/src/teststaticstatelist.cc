/*
 * teststaticstatelist.cc
 *
 *  Created on: Nov 5, 2012
 *      Author: slynen
 */


#include <loki/Typelist.h>
#include <Eigen/Dense>
#include <loki/HierarchyGenerators.h>
#include <sstream>
#include <iostream>
#include <loki/static_check.h>
#include <boost/preprocessor/punctuation/comma.hpp>

#define FUSION_MAX_VECTOR_SIZE 20

#include <boost/fusion/container/vector.hpp>
#include <boost/fusion/include/vector.hpp>
#include <boost/fusion/container/vector/vector_fwd.hpp>
#include <boost/fusion/include/vector_fwd.hpp>
#include <boost/fusion/sequence/intrinsic/size.hpp>
#include <boost/fusion/include/size.hpp>
#include <boost/fusion/sequence/intrinsic/at_c.hpp>
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

using namespace Loki::TL;

//a state variable with a name as specified in the state name enum
template<typename type_T, int name_T>
struct StateVar_T{
	typedef type_T state_T;
	enum{
		name_ = name_T
	};
	state_T state_;
};

//some tmp tools
namespace msf_tmp{

namespace{//hide these
template <typename T, typename U> struct SameType{enum { value = false };};
template <typename T> struct SameType<T,T>{enum { value = true };};

//the number of entries in the correction vector for a given state var
template<typename T>
struct CorrectionStateLengthForType;
template<int NAME> struct CorrectionStateLengthForType<const StateVar_T<Eigen::Vector3d, NAME>& >{
	enum{value = 3};
};
template<int NAME> struct CorrectionStateLengthForType<const StateVar_T<Eigen::Quaterniond, NAME>& >{
	enum{value = 3};
};
template<int NAME> struct CorrectionStateLengthForType<const StateVar_T<double, NAME>& >{
	enum{value = 1};
};
template<> struct CorrectionStateLengthForType<const mpl_::void_&>{
	enum{value = 0};
};

//the number of entries in the state for a given state var
template<typename T>
struct StateLengthForType;
template<int NAME> struct StateLengthForType<const StateVar_T<Eigen::Vector3d, NAME>& >{
	enum{value = 3};
};
template<int NAME> struct StateLengthForType<const StateVar_T<Eigen::Quaterniond, NAME>& >{
	enum{value = 4};
};
template<int NAME> struct StateLengthForType<const StateVar_T<double, NAME>& >{
	enum{value = 1};
};
template<> struct StateLengthForType<const mpl_::void_&>{
	enum{value = 0};
};

template <typename Sequence, typename First, typename Last, bool T>
struct countCorrectionStatesLinear;
template <typename Sequence, typename First, typename Last>
struct countCorrectionStatesLinear<Sequence, First, Last, true>{
	enum{//the end does not add entries
		value = 0
	};
};

template <class U> struct ReferenceTraits
{
	enum { result = false };
	typedef U ReferredType;
};

template <class U> struct ReferenceTraits<U&>
{
	enum { result = true };
	typedef U ReferredType;
};

template <typename Sequence, typename First, typename Last>
struct countCorrectionStatesLinear<Sequence, First, Last, false>{
	typedef typename boost::fusion::result_of::next<First>::type Next;
	typedef typename  boost::fusion::result_of::deref<First>::type current_Type;
	enum{//the length of the current state plus the tail of the list
		value = CorrectionStateLengthForType<current_Type>::value +
		countCorrectionStatesLinear<Sequence, Next, Last, SameType<First, Last>::value>::value
	};
};
}

template<typename Sequence>
struct CountCorrectionStates{
	typedef typename boost::fusion::result_of::begin<Sequence const>::type First;
	typedef typename boost::fusion::result_of::end<Sequence const>::type Last;
	enum{
		value = countCorrectionStatesLinear<Sequence, First, Last, SameType<First, Last>::value>::value
	};
};

//overloaded for all state types to apply corrections
template<typename T>
struct applycorrection
{
	applycorrection(T& correction):data_(correction){
		std::cout<<"Got correction vector"<<correction.transpose()<<std::endl;
	}
	template<int NAME>
	void operator()(StateVar_T<Eigen::Vector3d, NAME>& t) const{
		//TODO get index of the data in the correction vector
		std::cout<<"called correct for Vector3d"<<data_.transpose()<<std::endl;
		//		std::cout<<"value "<<t.state_<<std::endl;
	}
	template<int NAME>
	void operator()(StateVar_T<Eigen::Quaterniond, NAME>& t) const{
		std::cout<<"called correct for Quaterniond"<<data_.transpose()<<std::endl;
		//		std::cout<<"value "<<t.state_<<std::endl;
	}
	template<int NAME>
	void operator()(StateVar_T<double, NAME>& t) const{
		std::cout<<"called correct for double"<<data_.transpose()<<std::endl;
		//		std::cout<<"value "<<t.state_<<std::endl;
	}
private:
	T& data_;
};

}

template<typename stateVector_T>
struct EKFState_T{
	typedef stateVector_T state_T;
	stateVector_T states_;
	enum{
		nstates_ = boost::fusion::result_of::size<state_T>::type::value, //n all states
		ncorrectionstates_ = msf_tmp::CountCorrectionStates<state_T>::value //n correction states
	};

	//apply the correction vector to all state vars
	void correct(const Eigen::Matrix<double, ncorrectionstates_, 1>& correction) {
		boost::fusion::for_each(states_, msf_tmp::applycorrection<const Eigen::Matrix<double, ncorrectionstates_, 1> >(correction));
	}

	//checks whether the ordering in the vector is the same as in the enum,
	//which is something that strictly must not change
	template<int INDEX>
	void checkname(){
		BOOST_STATIC_ASSERT((INDEX ==
				boost::fusion::result_of::at<state_T, boost::mpl::int_<INDEX> >::type::name_));
	}

	//returns the state at position INDEX in the state list
	template<int INDEX>
	inline typename boost::fusion::result_of::at<state_T, boost::mpl::int_<INDEX> >::type::state_T& get(){
		return boost::fusion::result_of::at<state_T, boost::mpl::int_<INDEX> >::type::state_;
	}
};


int main(int argc, char** argv)
{
	enum{
		p_ci,
		q_ci,
		q_wv,
		L
	};

	//setup core state

	typedef boost::fusion::vector<
			StateVar_T<Eigen::Vector3d, p_ci >,
			StateVar_T<Eigen::Quaterniond, q_ci >,
			StateVar_T<double, L >
	> coreState_T;

	//aux state
	typedef boost::fusion::vector<
			StateVar_T<Eigen::Vector3d, p_ci >,
			StateVar_T<Eigen::Vector3d, q_ci >,
			StateVar_T<Eigen::Vector3d, q_wv >
	> auxState_T;

	//TODO: assemble complete state
	typedef coreState_T fullState_T;

	//an instantiation of a state
	EKFState_T<fullState_T> somestate;

	//number of states
	std::cout<<"nstates: "<<EKFState_T<fullState_T>::nstates_<<std::endl;

	//number of correction states
	std::cout<<"ncorrectionstates: "<<EKFState_T<fullState_T>::ncorrectionstates_<<std::endl;

	//apply correction to all states
	Eigen::Matrix<double, EKFState_T<fullState_T>::ncorrectionstates_, 1> correction;
	std::cout<<"passing random correction vector"<<std::endl;
	correction.setRandom();
	somestate.correct(correction);
//	std::cout<<"passing random correction vector"<<std::endl;
//	correction.setRandom();
//	somestate.correct(correction);




}
