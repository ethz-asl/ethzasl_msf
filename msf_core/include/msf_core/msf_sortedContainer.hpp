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

#ifndef MSF_SORTEDCONTAINER_HPP_
#define MSF_SORTEDCONTAINER_HPP_

#include <msf_core/msf_tools.hpp>

/***
 * A class managing a sorted container with strict less than ordering
 * used to store state and measurement objects which can then be queried
 * for closest states/measurements to a given time instant
 */
namespace msf_core{
template<typename T, typename PrototypeInvalidT = T>
class SortedContainer{

private:
	typedef std::map<double, boost::shared_ptr<T> > ListT; ///< the container type in which to store the data
	ListT stateList; ///< the container in which all the data is stored
	boost::shared_ptr<T> invalid; ///< a object to signal requests which cannot be satisfied
public:
	typedef typename ListT::iterator iterator_T;

	SortedContainer(){
		invalid.reset(new PrototypeInvalidT());
		invalid->time_ = -1;
	}
	/***
	 * returns an invalid object used to signal that a request could not be satisfied
	 */
	inline boost::shared_ptr<T>& getInvalid(){
		return invalid;
	}

	/***
	 * clears the internal container, dropping all the contents
	 */
	inline void clear(){
		stateList.clear();
	}

	/***
	 * returns the size of the internal container
	 */
	inline typename ListT::size_type size(){
		return stateList.size();
	}

	/***
	 * insert an object to the internal container to the position not violating the internal strict less than ordering by time
	 */
	inline typename ListT::iterator insert(const boost::shared_ptr<T>& value){
		std::pair<typename ListT::iterator,bool> itpr =
				stateList.insert(std::pair<double, boost::shared_ptr<T> >(value->time_, value));
		if(!itpr.second){
			ROS_WARN_STREAM("Wanted to insert a value to the sorted container at time "<<value->time_<<" but the map already contained a value at this time. discarding.");
		}
		return itpr.first;
	}

	/***
	 * returns the iterator at the beginning of the internal container
	 */
	inline typename ListT::iterator getIteratorBegin(){
		return stateList.begin();
	}

	/***
	 * returns the iterator at the end of the internal container
	 */
	inline typename ListT::iterator getIteratorEnd(){
		return stateList.end();
	}

	/***
	 * returns the iterator at the specific time instant of the supplied object
	 * or an invalid object if the request cannot be satisfied
	 */
	inline typename ListT::iterator getIteratorAtValue(const boost::shared_ptr<T>& value){
		typename ListT::iterator it = stateList.find(value->time_);
		if(it==stateList.end()){ //there is no value in the map with this time
			ROS_WARN_STREAM("getIteratorAtValue(state): Could not find value for time "<<value->time_<<"");
			it = stateList.lower_bound(value->time_);
		}
		return it;
	}


	/***
	 * returns the iterator at a specific time instant
	 * or an invalid object if the request cannot be satisfied
	 */
	inline typename ListT::iterator getIteratorAtValue(const double& time){
		typename ListT::iterator it = stateList.find(time);
		if(it==stateList.end()){ //there is no value in the map with this time
			ROS_WARN_STREAM("getIteratorAtValue(double): Could not find value for time "<<time<<"");
			it = stateList.lower_bound(time);
		}
		return it;
	}

	/***
	 * returns the iterator closest before a specific time instant
	 */
	inline typename ListT::iterator getIteratorClosestBefore(const double& statetime){
		typename ListT::iterator it = stateList.lower_bound(statetime);
		it--;
		return it;
	};

	/***
	 * returns the iterator closest after a specific time instant
	 */
	inline typename ListT::iterator getIteratorClosestAfter(const double& statetime){
		return  stateList.upper_bound(statetime);
	};

	/***
	 * returns the iterator closest to a specific time instant
	 */
	inline typename ListT::iterator getIteratorClosest(const double& statetime){
		typename ListT::iterator tauMinus = getIteratorClosestBefore(statetime);
		typename ListT::iterator tauPlus = getIteratorClosestAfter(statetime);

		typename ListT::iterator it_end = getIteratorEnd();
		if(tauMinus == it_end){
			return tauPlus;
		}
		if(tauPlus == it_end){
			return tauMinus;
		}
		if(fabs(tauPlus->second->time_ - statetime) < fabs(tauMinus->second->time_ - statetime)){
			return tauPlus;
		}else{
			return tauMinus;
		}
	}

	/***
	 * returns a pointer to the closest object before a specific time instant
	 */
	inline boost::shared_ptr<T>& getClosestBefore(const double& statetime){
		typename ListT::iterator it = stateList.lower_bound(statetime);
		if(it == stateList.begin()){
			return it->second;
		}
		it--;
		return it->second;
	};

	/***
	 * returns a pointer to the closest after a specific time instant
	 * or an invalid object if the request cannot be satisfied
	 */
	inline boost::shared_ptr<T>& getClosestAfter(const double& statetime){
		typename ListT::iterator it = stateList.upper_bound(statetime);
		if(it==stateList.end()){
			return getInvalid();
		}
		return  it->second;
	};

	/***
	 * returns a pointer to the object at a specific time instant
	 * or an invalid object if the request cannot be satisfied
	 */
	inline boost::shared_ptr<T>& getValueAt(const double& statetime){
		typename ListT::iterator it = stateList.find(statetime);
		if(it==stateList.end()){
			return getInvalid();
		}
		return  it->second;
	};

	/***
	 * returns a pointer to the closest to a specific time instant
	 */
	inline boost::shared_ptr<T>& getClosest(const double& statetime){
		boost::shared_ptr<T>& at = getValueAt(statetime); //is there one exactly at this point?
		if(at != getInvalid()){
			return at;
		}

		boost::shared_ptr<T>& tauMinus = getClosestBefore(statetime);
		boost::shared_ptr<T>& tauPlus = getClosestAfter(statetime);

		if(tauMinus->time_==-1){
			return tauPlus;
		}else if(tauPlus->time_==-1){
			return tauMinus;
		}

		if(fabs(tauPlus->time_ - statetime) < fabs(tauMinus->time_ - statetime)){
			return tauPlus;
		}else{
			return tauMinus;
		}
	}

	/***
	 * Clears all objects having a time stamp older than the supplied time in seconds
	 */
	inline void clearOlderThan(double age){
		double newest = getLast()->time_;
		iterator_T it = getIteratorClosest(newest-age);
		if(newest - it->second->time_ < age)
			return; //there is no state older than time
		if(it->second->time_ > stateList.begin()->second->time_)
			stateList.erase(stateList.begin(),it);
	}

	/***
	 * returns a pointer to the last object in the container
	 * or an invalid object if the container is empty
	 */
	inline boost::shared_ptr<T>& getLast(){
		if(stateList.empty()){
			ROS_ERROR_STREAM("requested the last object in the sorted container, but the container is empty");
			return getInvalid();
		}
		typename ListT::iterator end = stateList.end();
		return (--end)->second;
	}

	/***
	 * This function updates the time of an object in the container
	 * this function effectively changes the map ordering, so the previous iterators are invalidated
	 * the attribute unused can be eliminated for non gcc compilers, its just a little more verbose
	 * in cases where the updated value is not used
	 */
	inline boost::shared_ptr<T> updateTime(double timeOld, double timeNew) __attribute__ ((warn_unused_result)) {
		typename ListT::iterator it = stateList.find(timeOld);
		if(it == stateList.end()){
			std::stringstream ss;
			ss<<"Wanted to update a states/measurements time, but could not find the old state, "
					"for which the time was asked to be updated. time "<<msf_core::timehuman(timeOld)<<std::endl;

			ss<<"Map: "<<std::endl;
			for(typename ListT::iterator it2 = stateList.begin(); it2!= stateList.end(); ++it2){
				ss<<it2->first<<std::endl;
			}
			ROS_ERROR_STREAM_THROTTLE(1,ss.str());

			return getClosest(timeOld);
		}
		boost::shared_ptr<T> copy = it->second; //get the data from the map, we need to update, then reinsert
		stateList.erase(it->first);
		copy->time_ = timeNew;
		typename ListT::iterator inserted = insert(copy);
		return inserted->second;
	}

	/***
	 * debug output the contents of the container in a human readable time format
	 */
	std::string echoBufferContentTimes(){
		std::stringstream ss;

		for(typename ListT::iterator it = getIteratorBegin();it!=getIteratorEnd();++it){
			ss<<msf_core::timehuman(it->second->time_)<<std::endl;
		}
		return ss.str();

	}

};
}

#endif /* MSF_SORTEDCONTAINER_HPP_ */
