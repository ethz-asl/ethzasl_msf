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

#include <msf_core/msf_tools.h>

namespace msf_core{
/**
 * \brief Manages a sorted container with strict less than ordering
 * used to store state and measurement objects which can then be queried
 * for closest states/measurements to a given time instant
 */
template<typename T, typename PrototypeInvalidT = T>
class SortedContainer{

public:
  typedef boost::shared_ptr<T> Ptr_T;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

private:
  typedef std::map<double, Ptr_T > ListT; ///< the container type in which to store the data
  ListT stateList; ///< the container in which all the data is stored
  Ptr_T invalid; ///< a object to signal requests which cannot be satisfied
public:
  typedef typename ListT::iterator iterator_T;

  SortedContainer(){
    invalid.reset(new PrototypeInvalidT());
    invalid->time = -1;
  }
  /**
   * \brief to be called to signal that a request could not be satisfied
   * \returns an object of the "invalid" type
   */
  inline boost::shared_ptr<T>& getInvalid(){
    return invalid;
  }

  /**
   * \brief clears the internal container, dropping all the contents
   */
  inline void clear(){
    stateList.clear();
  }

  /**
   * \brief returns the size of the internal container
   * \returns size of the container
   */
  inline typename ListT::size_type size(){
    return stateList.size();
  }

  /**
   * \brief insert an object to the internal container to the position not violating the internal strict less than ordering by time
   */
  inline typename ListT::iterator insert(const boost::shared_ptr<T>& value){
    std::pair<typename ListT::iterator,bool> itpr =
        stateList.insert(std::pair<double, boost::shared_ptr<T> >(value->time, value));
    if(!itpr.second){
      ROS_WARN_STREAM("Wanted to insert a value to the sorted container at time "<<std::fixed<<std::setprecision(9)<<value->time<<" but the map already contained a value at this time. discarding.");
    }
    return itpr.first;
  }

  /**
   * \brief returns the iterator at the beginning of the internal container
   */
  inline typename ListT::iterator getIteratorBegin(){
    return stateList.begin();
  }

  /**
   * \brief returns the iterator before the beginning of the internal container
   */
  inline typename ListT::iterator getIteratorBeforeBegin(){
    typename ListT::iterator it =  stateList.begin();
    return --it;
  }

  /**
   * \brief returns the iterator at the end of the internal container
   */
  inline typename ListT::iterator getIteratorEnd(){
    return stateList.end();
  }

  /**
   * \brief returns the iterator at the specific time instant of the supplied object
   * or an invalid object if the request cannot be satisfied
   * \param value the value to get the iterator for
   * \returns iterator
   */
  inline typename ListT::iterator getIteratorAtValue(const boost::shared_ptr<T>& value, bool warnIfNotExistant = true){
    typename ListT::iterator it = stateList.find(value->time);
    if(it==stateList.end()){ //there is no value in the map with this time
      if(warnIfNotExistant)
        ROS_WARN_STREAM("getIteratorAtValue(state): Could not find value for time "<<std::fixed<<std::setprecision(9)<<value->time<<"");
      it = stateList.lower_bound(value->time);
    }
    return it;
  }


  /**
   * \brief returns the iterator at a specific time instant
   * or an invalid object if the request cannot be satisfied
   * \param time the time where we want to get an iterator at
   * \returns iterator
   */
  inline typename ListT::iterator getIteratorAtValue(const double& time, bool warnIfNotExistant = true){
    typename ListT::iterator it = stateList.find(time);
    if(it==stateList.end()){ //there is no value in the map with this time
      if(warnIfNotExistant)
        ROS_WARN_STREAM("getIteratorAtValue(double): Could not find value for time "<<std::fixed<<std::setprecision(9)<<time<<"");
      it = stateList.lower_bound(time);
    }
    return it;
  }

  /**
   * \brief returns the iterator closest before a specific time instant
   * \param time the time where we want to get an iterator at
   * \returns iterator
   */
  inline typename ListT::iterator getIteratorClosestBefore(const double& statetime){
    typename ListT::iterator it = stateList.lower_bound(statetime);
    it--;
    return it;
  };

  /**
   * \brief returns the iterator closest after a specific time instant
   * \param time the time where we want to get an iterator at
   * \returns iterator
   */
  inline typename ListT::iterator getIteratorClosestAfter(const double& statetime){
    return  stateList.upper_bound(statetime);
  };

  /**
   * \brief returns the iterator closest to a specific time instant
   * \param time the time where we want to get an iterator at
   * \returns iterator
   */
  inline typename ListT::iterator getIteratorClosest(const double& statetime){

    //first check if we have a value at this time in the buffer
    typename ListT::iterator it_at = stateList.find(statetime);
    if(it_at != stateList.end()){
      return it_at;
    }

    //if not, find the closest one
    typename ListT::iterator tauMinus = getIteratorClosestBefore(statetime);
    typename ListT::iterator tauPlus = getIteratorClosestAfter(statetime);

    typename ListT::iterator it_end = getIteratorEnd();
    if(tauMinus == it_end){
      return tauPlus;
    }
    if(tauPlus == it_end){
      return tauMinus;
    }
    if(fabs(tauPlus->second->time - statetime) < fabs(tauMinus->second->time - statetime)){
      return tauPlus;
    }else{
      return tauMinus;
    }
  }

  /**
   * \brief returns a pointer to the closest object before a specific time instant
   * \param time the time where we want to get the value at
   * \returns shared pointer of the object
   */
  inline boost::shared_ptr<T>& getClosestBefore(const double& statetime){
    typename ListT::iterator it = stateList.lower_bound(statetime);
    if(it == stateList.begin()){
      return it->second;
    }
    it--;
    return it->second;
  };

  /**
   * \brief returns a pointer to the closest after a specific time instant
   * or an invalid object if the request cannot be satisfied
   * \param time the time where we want to get the value at
   * \returns shared pointer of the object
   */
  inline boost::shared_ptr<T>& getClosestAfter(const double& statetime){
    typename ListT::iterator it = stateList.upper_bound(statetime);
    if(it==stateList.end()){
      return getInvalid();
    }
    return  it->second;
  };

  /**
   * \brief returns a pointer to the object at a specific time instant
   * or an invalid object if the request cannot be satisfied
   * \param time the time where we want to get the value at
   * \returns shared pointer of the object
   */
  inline boost::shared_ptr<T>& getValueAt(const double& statetime){
    typename ListT::iterator it = stateList.find(statetime);
    if(it==stateList.end()){
      return getInvalid();
    }
    return  it->second;
  };

  /**
   * \brief returns a pointer to the closest to a specific time instant
   * \param time the time where we want to get the value at
   * \returns shared pointer of the object
   */
  inline boost::shared_ptr<T>& getClosest(const double& statetime){
    boost::shared_ptr<T>& at = getValueAt(statetime); //is there one exactly at this point?
    if(at != getInvalid()){
      return at;
    }

    boost::shared_ptr<T>& tauMinus = getClosestBefore(statetime);
    boost::shared_ptr<T>& tauPlus = getClosestAfter(statetime);

    if(tauMinus->time==-1){
      return tauPlus;
    }else if(tauPlus->time==-1){
      return tauMinus;
    }

    if(fabs(tauPlus->time - statetime) < fabs(tauMinus->time - statetime)){
      return tauPlus;
    }else{
      return tauMinus;
    }
  }

  /**
   * \brief Clears all objects having a time stamp older than the supplied time in seconds
   * \param time the maximum age of states in the container
   * \returns shared pointer of the object
   */
  inline void clearOlderThan(double age){
    double newest = getLast()->time;
    iterator_T it = getIteratorClosest(newest-age);
    if(newest - it->second->time < age)
      return; //there is no state older than time
    if(it->second->time > stateList.begin()->second->time)
      stateList.erase(stateList.begin(),it);
  }

  /**
   * \brief returns a pointer to the last object in the container
   * or an invalid object if the container is empty
   * \returns shared pointer of the object
   */
  inline boost::shared_ptr<T>& getLast(){
    if(stateList.empty()){
      ROS_ERROR_STREAM("requested the last object in the sorted container, but the container is empty");
      return getInvalid();
    }
    typename ListT::iterator end = stateList.end();
    return (--end)->second;
  }

  /**
   * \brief returns a pointer to the first object in the container
   * or an invalid object if the container is empty
   * \returns shared pointer of the object
   */
  inline boost::shared_ptr<T>& getFirst(){
    if(stateList.empty()){
      ROS_ERROR_STREAM("requested the first object in the sorted container, but the container is empty");
      return getInvalid();
    }
    typename ListT::iterator start = stateList.begin();
    return start->second;
  }

  /**
   * \brief This function updates the time of an object in the container
   * this function effectively changes the map ordering, so the previous iterators are invalidated
   * the attribute unused can be eliminated for non gcc compilers, its just a little more verbose
   * in cases where the updated value is not used
   * \param timeOld the time of the value to update
   * \param timeNew the time to update to
   * \returns shared pointer of the object
   */
  inline boost::shared_ptr<T> updateTime(double timeOld, double timeNew) __attribute__ ((warn_unused_result)) {
    typename ListT::iterator it = stateList.find(timeOld);
    if(it == stateList.end()){
      std::stringstream ss;
      ss<<"Wanted to update a states/measurements time, but could not find the old state, "
          "for which the time was asked to be updated. time "<<std::fixed<<std::setprecision(9)<<timeOld<<std::endl;

      ss<<"Map: "<<std::endl;
      for(typename ListT::iterator it2 = stateList.begin(); it2!= stateList.end(); ++it2){
        ss<<it2->first<<std::endl;
      }
      ROS_ERROR_STREAM_THROTTLE(1,ss.str());

      return getClosest(timeOld);
    }
    boost::shared_ptr<T> copy = it->second; //get the data from the map, we need to update, then reinsert
    stateList.erase(it->first);
    copy->time = timeNew;
    typename ListT::iterator inserted = insert(copy);
    return inserted->second;
  }

  /**
   * \brief debug output the contents of the container in a human readable time format
   * \returns string of the buffer contents with line breaks after every entry
   */
  std::string echoBufferContentTimes(){
    std::stringstream ss;

    for(typename ListT::iterator it = getIteratorBegin();it!=getIteratorEnd();++it){
      ss<<std::fixed<<std::setprecision(9)<<it->second->time<<std::endl;
    }
    return ss.str();

  }

};
}

#endif /* MSF_SORTEDCONTAINER_HPP_ */
