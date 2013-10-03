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
#ifndef MSF_SORTEDCONTAINER_H_
#define MSF_SORTEDCONTAINER_H_

#include <msf_core/msf_types.hpp>
#include <msf_core/msf_tools.h>
#include <msf_core/msf_macros.h>
#include <iomanip>

#define CHECK_IN_BOUNDS(iterator, container) \
           do { \
             decltype(iterator) __it = it; \
             ++__it; \
             if (__it == container.begin()) { \
               MSF_ERROR_STREAM("Iterator out of bounds (begin) " << \
                 __FILE__ << ":" << __LINE__); \
             } \
             if (iterator == container.end()) { \
               MSF_ERROR_STREAM("Iterator out of bounds (end) " << \
                 __FILE__ << ":" << __LINE__); \
             } \
           } while(0);


namespace msf_core {
/**
 * \brief Manages a sorted container with strict less than ordering
 * used to store state and measurement objects which can then be queried
 * for closest states/measurements to a given time instant
 */
template<typename T, typename PrototypeInvalidT = T>
class SortedContainer {

 public:
  typedef shared_ptr<T> Ptr_T;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  typedef std::map<double, Ptr_T> ListT;  ///< The container type in which to store the data.
  ListT stateList;  ///< The container in which all the data is stored.
  Ptr_T invalid;  ///< A object to signal requests which cannot be satisfied.
 public:
  typedef typename ListT::iterator iterator_T;

  SortedContainer() {
    invalid.reset(new PrototypeInvalidT());
    invalid->time = -1;
  }
  /**
   * \brief To be called to signal that a request could not be satisfied
   * \returns an object of the "invalid" type
   */
  inline shared_ptr<T>& getInvalid() {
    return invalid;
  }

  /**
   * \brief Clears the internal container, dropping all the contents.
   */
  inline void clear() {
    stateList.clear();
  }

  /**
   * \brief Returns the size of the internal container.
   * \returns Size of the container.
   */
  inline typename ListT::size_type size() {
    return stateList.size();
  }

  /**
   * \brief Insert an object to the internal container to the position not
   * violating the internal strict less than ordering by time.
   */
  inline typename ListT::iterator insert(const shared_ptr<T>& value) {
    std::pair<typename ListT::iterator, bool> itpr = stateList.insert(
        std::pair<double, shared_ptr<T> >(value->time, value));
    if (!itpr.second) {
      MSF_WARN_STREAM(
          "Wanted to insert a value to the sorted container at time " <<
          std::fixed << std::setprecision(9) << value->time <<
          " but the map already contained a value at this time. discarding.");
    }
    return itpr.first;
  }

  /**
   * \brief Returns the iterator at the beginning of the internal container.
   */
  inline typename ListT::iterator getIteratorBegin() {
    return stateList.begin();
  }

  /**
   * \brief Returns the iterator before the beginning of the internal container.
   */
  inline typename ListT::iterator getIteratorBeforeBegin() {
    typename ListT::iterator it = stateList.begin();
    return --it;
  }

  /**
   * \brief Returns the iterator at the end of the internal container.
   */
  inline typename ListT::iterator getIteratorEnd() {
    return stateList.end();
  }

  /**
   * \brief Returns the iterator at the specific time instant of the supplied
   * object or an invalid object if the request cannot be satisfied.
   * \param value The value to get the iterator for.
   * \returns iterator.
   */
  inline typename ListT::iterator getIteratorAtValue(const shared_ptr<T>& value,
                                                     bool warnIfNotExistant =
                                                         true) {
    typename ListT::iterator it = stateList.find(value->time);
    if (it == stateList.end()) {  // There is no value in the map with this time.
      if (warnIfNotExistant)
        MSF_WARN_STREAM(
            "getIteratorAtValue(state): Could not find value for time " <<
            std::fixed << std::setprecision(9) << value->time);
      it = stateList.lower_bound(value->time);
    }
    return it;
  }

  /**
   * \brief Returns the iterator at a specific time instant
   * or an invalid object if the request cannot be satisfied.
   * \param time The time where we want to get an iterator at.
   * \returns iterator.
   */
  inline typename ListT::iterator getIteratorAtValue(const double& time,
                                                     bool warnIfNotExistant =
                                                         true) {
    typename ListT::iterator it = stateList.find(time);
    if (it == stateList.end()) {  //there is no value in the map with this time
      if (warnIfNotExistant)
        MSF_WARN_STREAM(
            "getIteratorAtValue(double): Could not find value for time " <<
            std::fixed << std::setprecision(9) << time);
      it = stateList.lower_bound(time);
    }
    return it;
  }

  /**
   * \brief Returns the iterator closest before a specific time instant
   * \param time The time where we want to get an iterator at.
   * \returns iterator.
   */
  inline typename ListT::iterator getIteratorClosestBefore(
      const double& statetime) {
    typename ListT::iterator it = stateList.lower_bound(statetime);
    it--;
    return it;
  }

  /**
   * \brief Returns the iterator closest after a specific time instant.
   * \param time The time where we want to get an iterator at.
   * \returns iterator.
   */
  inline typename ListT::iterator getIteratorClosestAfter(
      const double& statetime) {
    typename ListT::iterator it = stateList.upper_bound(statetime);
    return it;
  }

  /**
   * \brief Returns the iterator closest to a specific time instant.
   * \param time The time where we want to get an iterator at.
   * \returns iterator.
   */
  inline typename ListT::iterator getIteratorClosest(const double& statetime) {

    // First check if we have a value at this time in the buffer.
    typename ListT::iterator it_at = stateList.find(statetime);
    if (it_at != stateList.end()) {
      return it_at;
    }

    // If not, find the closest one.
    typename ListT::iterator tauMinus = getIteratorClosestBefore(statetime);
    typename ListT::iterator tauPlus = getIteratorClosestAfter(statetime);

    typename ListT::iterator it_end = getIteratorEnd();
    if (tauMinus == it_end) {
      return tauPlus;
    }
    if (tauPlus == it_end) {
      return tauMinus;
    }
    if (fabs(tauPlus->second->time - statetime)
        < fabs(tauMinus->second->time - statetime)) {
      return tauPlus;
    } else {
      return tauMinus;
    }
  }

  /**
   * \brief Returns a pointer to the closest object before a specific time instant.
   * \param Time the time where we want to get the value at.
   * \returns shared pointer of the object.
   */
  inline shared_ptr<T>& getClosestBefore(const double& statetime) {
    typename ListT::iterator it = stateList.lower_bound(statetime);
    if (stateList.empty()) {
      MSF_WARN_STREAM("Requested the first object before time " << statetime <<
        "but the container is empty");
      return getInvalid();
    }
    if (it == stateList.begin()) {
      return it->second;
    }
    it--;
    return it->second;
  }
  ;

  /**
   * \brief Returns a pointer to the closest after a specific time instant
   * or an invalid object if the request cannot be satisfied.
   * \param time The time where we want to get the value at
   * \returns shared pointer of the object.
   */
  inline shared_ptr<T>& getClosestAfter(const double& statetime) {
    typename ListT::iterator it = stateList.upper_bound(statetime);
    if (it == stateList.end()) {
      return getInvalid();
    }
    return it->second;
  }
  ;

  /**
   * \brief Returns a pointer to the object at a specific time instant
   * or an invalid object if the request cannot be satisfied.
   * \param time The time where we want to get the value at.
   * \returns shared pointer of the object.
   */
  inline shared_ptr<T>& getValueAt(const double& statetime) {
    typename ListT::iterator it = stateList.find(statetime);
    if (it == stateList.end()) {
      return getInvalid();
    }
    return it->second;
  }
  ;

  /**
   * \brief Returns a pointer to the closest to a specific time instant.
   * \param time The time where we want to get the value at.
   * \returns shared pointer of the object.
   */
  inline shared_ptr<T>& getClosest(const double& statetime) {
    shared_ptr<T>& at = getValueAt(statetime);  // Is there one exactly at this point?
    if (at != getInvalid()) {
      return at;
    }

    shared_ptr<T>& tauMinus = getClosestBefore(statetime);
    shared_ptr<T>& tauPlus = getClosestAfter(statetime);

    if (tauMinus->time == -1) {
      return tauPlus;
    } else if (tauPlus->time == -1) {
      return tauMinus;
    }

    if (fabs(tauPlus->time - statetime) < fabs(tauMinus->time - statetime)) {
      return tauPlus;
    } else {
      return tauMinus;
    }
  }

  /**
   * \brief Clears all objects having a time stamp older than the supplied time
   * in seconds.
   * \param time The maximum age of states in the container.
   * \returns shared pointer of the object.
   */
  inline void clearOlderThan(double age) {
    double newest = getLast()->time;
    iterator_T it = getIteratorClosest(newest - age);
    if (newest - it->second->time < age)
      return;  //there is no state older than time
    if (it->second->time > stateList.begin()->second->time)
      stateList.erase(stateList.begin(), it);
  }

  /**
   * \brief Returns a pointer to the last object in the container
   * or an invalid object if the container is empty.
   * \returns shared pointer of the object.
   */
  inline shared_ptr<T>& getLast() {
    if (stateList.empty()) {
      MSF_WARN_STREAM("Requested the last object in the sorted container, but "
      "the container is empty");
      return getInvalid();
    }
    typename ListT::iterator end = stateList.end();
    return (--end)->second;
  }

  /**
   * \brief Returns a pointer to the first object in the container
   * or an invalid object if the container is empty.
   * \returns shared pointer of the object.
   */
  inline shared_ptr<T>& getFirst() {
    if (stateList.empty()) {
      MSF_WARN_STREAM("Requested the first object in the sorted container, "
      "but the container is empty");
      return getInvalid();
    }
    typename ListT::iterator start = stateList.begin();
    return start->second;
  }

  /**
   * \brief This function updates the time of an object in the container
   * this function effectively changes the map ordering, so the previous
   * iterators are invalidated the attribute unused can be eliminated for non
   * gcc compilers, its just a little more verbose in cases where the updated
   * value is not used.
   * \param timeOld The time of the value to update.
   * \param timeNew The time to update to.
   * \returns shared pointer of the object.
   */
  inline shared_ptr<T> updateTime(double timeOld, double timeNew)
      __attribute__ ((warn_unused_result)) {
    typename ListT::iterator it = stateList.find(timeOld);
    if (it == stateList.end()) {
      std::stringstream ss;
      ss
          << "Wanted to update a states/measurements time, but could not find "
              "the old state, for which the time was asked to be updated. time "
          << std::fixed << std::setprecision(9) << timeOld << std::endl;

      ss << "Map: " << std::endl;
      for (typename ListT::iterator it2 = stateList.begin();
          it2 != stateList.end(); ++it2) {
        ss << it2->first << std::endl;
      }
      MSF_WARN_STREAM(ss.str());

      return getClosest(timeOld);
    }
    // Get the data from the map, we need to update, then reinsert.
    shared_ptr<T> copy = it->second;
    stateList.erase(it->first);
    copy->time = timeNew;
    typename ListT::iterator inserted = insert(copy);
    return inserted->second;
  }

  /**
   * \brief Debug output the contents of the container in a human readable time
   * format.
   * \returns String of the buffer contents with line breaks after every entry.
   */
  std::string echoBufferContentTimes() {
    std::stringstream ss;

    for (typename ListT::iterator it = getIteratorBegin();
        it != getIteratorEnd(); ++it) {
      ss << std::fixed << std::setprecision(9) << it->second->time << std::endl;
    }
    return ss.str();

  }

};
}

#endif  // MSF_SORTEDCONTAINER_H_
