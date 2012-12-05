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

	inline void clear(){
		stateList.clear();
	}

	inline typename ListT::iterator insert(const boost::shared_ptr<T>& value){
		std::pair<typename ListT::iterator,bool> itpr =
				stateList.insert(std::pair<double, boost::shared_ptr<T> >(value->time_, value));
		if(!itpr.second){
			ROS_WARN_STREAM("Wanted to insert a measurement at time "<<value->time_<<" but the map already contained a measurement at this time. discarding.");
		}
		return itpr.first;
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
			ROS_WARN_STREAM("getIteratorAtValue(state): Could not find value for time "<<value->time_<<"");
			it = stateList.lower_bound(value->time_);
		}
		return it;
	}


	inline typename ListT::iterator getIteratorAtValue(const double& time){
		typename ListT::iterator it = stateList.find(time);
		if(it==stateList.end()){ //this state is not known
			ROS_WARN_STREAM("getIteratorAtValue(double): Could not find value for time "<<time<<"");
			it = stateList.lower_bound(time);
		}
		return it;
	}

	inline typename ListT::iterator getIteratorClosestBefore(const double& statetime){
		typename ListT::iterator it = stateList.lower_bound(statetime);
		it--;
		return it;
	};

	inline typename ListT::iterator getIteratorClosestAfter(const double& statetime){
		return  stateList.upper_bound(statetime);
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
		if(it == stateList.begin()){
			return it->second;
		}
		it--;
		return it->second;
	};

	inline boost::shared_ptr<T>& getClosestAfter(const double& statetime){
		typename ListT::iterator it = stateList.upper_bound(statetime);
		if(it==stateList.end()){
			return getInvalid();
		}
		return  it->second;
	};

	inline boost::shared_ptr<T>& getClosest(const double& statetime){
		boost::shared_ptr<T>& tauMinus = getClosestBefore(statetime);
		boost::shared_ptr<T>& tauPlus = getClosestAfter(statetime);
		//TODO remove in production code{
		if(tauMinus->time_==-1&&tauPlus->time_==-1){
			std::cout<<"ERROR neighter getClosestBefore nor getClosestAfter returned a valid value"<<std::endl;
		}
		if(tauMinus->time_==-1){
			return tauPlus;
		}else if(tauPlus->time_==-1){
			return tauMinus;
		}
		//		double tdiff1 = fabs(tauPlus->time_ - statetime);
		//		double tdiff2 = fabs(tauMinus->time_ - statetime);
		//		ROS_INFO_STREAM("Requested a state close to "<<statetime<<" dt1 "<<tdiff1<<" dt2 "<<tdiff2);
		//}

		if(fabs(tauPlus->time_ - statetime) < fabs(tauMinus->time_ - statetime)){
			return tauPlus;
		}else{
			return tauMinus;
		}
	}

	inline boost::shared_ptr<T>& getLast(){
		typename ListT::iterator end = stateList.end();
		return (--end)->second;
	}

	//this function effectively changes the map ordering, so the previous iterators are invalidated
	inline boost::shared_ptr<T> updateTime(double timeOld, double timeNew) /*__attribute__ ((warn_unused_result))*/ {
		typename ListT::iterator it = stateList.find(timeOld);
		if(it == stateList.end()){
			std::stringstream ss;
			ss<<"Wanted to update a states/measurements time, but could not find the old state, "
					"for which the time was asked to be updated. time "<<timeOld<<std::endl;

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


};
}

#endif /* MSF_SORTEDCONTAINER_HPP_ */
