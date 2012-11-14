/*
 * msf_sortedContainer.hpp
 *
 *  Created on: Nov 13, 2012
 *      Author: slynen
 */

#ifndef MSF_SORTEDCONTAINER_HPP_
#define MSF_SORTEDCONTAINER_HPP_

namespace msf_core{
template<typename T, typename PrototypeInvalidT = T>
class SortedContainer{

private:
	typedef std::map<double, boost::shared_ptr<T> > ListT;
	ListT stateList;
	boost::shared_ptr<T> invalid;
public:
	typedef typename ListT::iterator iterator_T;

	SortedContainer(){
		invalid.reset(new PrototypeInvalidT());
		invalid->time_ = -1;
	}

	inline boost::shared_ptr<T>& getInvalid(){
		return invalid;
	}

	inline typename ListT::iterator insert(const boost::shared_ptr<T>& value){
		std::pair<typename ListT::iterator,bool> it =
				stateList.insert(std::pair<double, boost::shared_ptr<T> >(value->time_, value));
		if(!it.second){
			ROS_WARN_STREAM("Wanted to insert a measurement at time "<<value->time_<<" but the map already contained a measurement at this time. discarding.");
		}
		return it.first;
	}

	inline typename ListT::iterator getIteratorBegin(){
		return stateList.begin();
	}

	inline typename ListT::iterator getIteratorEnd(){
		return stateList.end();
	}

	inline typename ListT::iterator getIteratorAtValue(const boost::shared_ptr<T>& value){
		typename ListT::iterator it = stateList.find(value->time_);
		if(it==stateList.end()){ //this state is not known
			ROS_WARN_STREAM("getIteratorAtValue: Could not find value for time "<<value->time_<<"");
			it = stateList.lower_bound(value->time_);
		}
		return it;
	}

	inline typename ListT::iterator getIteratorClosestBefore(const double& statetime){
		typename ListT::iterator it = stateList.lower_bound(statetime);
		it--;
		return it;
	};

	inline typename ListT::iterator getIteratorClosestAfter(const double& statetime){
		return  stateList.lower_bound(statetime);
	};

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
		if(abs(tauPlus->second->time_ - statetime) < abs(tauMinus->second->time_ - statetime)){
			return tauPlus;
		}else{
			return tauMinus;
		}
	}

	inline boost::shared_ptr<T>& getClosestBefore(const double& statetime){
		typename ListT::iterator it = stateList.lower_bound(statetime);
		if(it == stateList.end()){
			return getInvalid();
		}
		it--;
		if(it == stateList.end()){
			return getInvalid();
		}
		return it->second;
	};

	inline boost::shared_ptr<T>& getClosestAfter(const double& statetime){
		typename ListT::iterator it = stateList.lower_bound(statetime);
		if(it==stateList.end()){
			return getInvalid();
		}
		return  it->second;
	};

	inline boost::shared_ptr<T>& getClosest(const double& statetime){
		boost::shared_ptr<T>& tauMinus = getClosestBefore(statetime);
		boost::shared_ptr<T>& tauPlus = getClosestAfter(statetime);
		if(abs(tauPlus->time_ - statetime) < abs(tauMinus->time_ - statetime)){
			return tauPlus;
		}else{
			return tauMinus;
		}
	}

	inline boost::shared_ptr<T>& getLast(){
		typename ListT::iterator end = stateList.end();
		return (--end)->second;
	}

	//this function effectively changes the map ordering
	inline void updateTime(double timeOld, double timeNew){
		typename ListT::iterator it = stateList.find(timeOld);
		if(it == stateList.end()){
			ROS_WARN_STREAM("Wanted to update a states/measurements time, but could not find the old state, for which the time was asked to be updated");
		}
		boost::shared_ptr<T> copy = it->second; //get the data from the map, we need to update, then reinsert
		stateList.erase(it);
		copy->time_ = timeNew;
		insert(copy);
	}


};
}

#endif /* MSF_SORTEDCONTAINER_HPP_ */
