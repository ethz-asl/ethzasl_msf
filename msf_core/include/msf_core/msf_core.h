/*

Copyright (c) 2010, Stephan Weiss, ASL, ETH Zurich, Switzerland
You can contact the author at <stephan dot weiss at ieee dot org>
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

#ifndef MSF_CORE_H_
#define MSF_CORE_H_


#include <Eigen/Eigen>

#include <ros/ros.h>

// message includes
#include <sensor_fusion_comm/DoubleArrayStamped.h>
#include <sensor_fusion_comm/ExtState.h>
#include <sensor_fusion_comm/ExtEkf.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>

#include <msf_core/msf_sortedContainer.h>
#include <vector>
#include <queue>
#include <msf_core/msf_state.h>
#include <msf_core/msf_checkFuzzyTracking.h>

//good old days...
//#define N_STATE_BUFFER 256	///< size of unsigned char, do not change!

namespace msf_core{

enum{
  HLI_EKF_STATE_SIZE = 16 ///< number of states exchanged with external propagation. Here: p,v,q,bw,bw=16
};

template<typename EKFState_T>
class MSF_SensorManager;

/** \class MSF_Core
 *
 * \brief The core class of the EKF
 * Does propagation of state and covariance
 * but also applying measurements and managing states and measurements
 * in lists sorted by time stamp
 */
template<typename EKFState_T>
class MSF_Core
{
  friend class MSF_MeasurementBase<EKFState_T>;
  bool initialized_; ///< is the filter initialized, so that we can propagate the state?
  bool predictionMade_; ///< is there a state prediction, so we can apply measurements?
public:
  typedef typename EKFState_T::StateDefinition_T StateDefinition_T;
  typedef typename EKFState_T::StateSequence_T StateSequence_T;
  enum{
    nErrorStatesAtCompileTime = EKFState_T::nErrorStatesAtCompileTime,  ///< error state length
    nStatesAtCompileTime = EKFState_T::nStatesAtCompileTime ///< complete state length
  };
  typedef Eigen::Matrix<double, nErrorStatesAtCompileTime, 1> ErrorState; ///< the error state type
  typedef Eigen::Matrix<double, nErrorStatesAtCompileTime, nErrorStatesAtCompileTime> ErrorStateCov; ///<the error state covariance type

  typedef msf_core::SortedContainer<EKFState_T> stateBufferT; ///< the type of the state buffer containing all the states
  typedef msf_core::SortedContainer<typename msf_core::MSF_MeasurementBase<EKFState_T>, typename msf_core::MSF_InvalidMeasurement<EKFState_T> > measurementBufferT; ///< the type of the measurement buffer containing all the measurements

  /**
   * \brief add a sensor measurement or an init measurement to the internal queue and apply it to the state
   * \param measurement the measurement to add to the internal measurement queue
   */
  void addMeasurement(boost::shared_ptr<MSF_MeasurementBase<EKFState_T> > measurement);

  /**
   * \brief initializes the filter with the values of the given measurement, other init values from other
   * sensors can be passed in as "measurement" using the initMeasurement structs
   * \param measurement a measurement containing initial values for the state
   */
  void init(boost::shared_ptr<MSF_MeasurementBase<EKFState_T> > measurement);

  /**
   * \brief initialize the HLP based propagation
   * \param state the state to send to the HLP
   */
  void initExternalPropagation(boost::shared_ptr<EKFState_T> state);

  /**
   * \brief finds the closest state to the requested time in the internal state
   * \param tstamp the time stamp to find the closest state to
   */
  boost::shared_ptr<EKFState_T> getClosestState(double tstamp);

  /**
   * \brief propagates the error state covariance
   * \param state_old the state to propagate the covariance from
   * \param state_new the state to propagate the covariance to
   */
  void predictProcessCovariance(boost::shared_ptr<EKFState_T>& state_old, boost::shared_ptr<EKFState_T>& state_new);

  /**
   * \brief delete very old states and measurements from the buffers to free memory
   */
  void CleanUpBuffers(){
    double timeold = 60; //1 min
    StateBuffer_.clearOlderThan(timeold);
    MeasurementBuffer_.clearOlderThan(timeold);
  }

  /**
   * \brief sets the covariance matrix of the core states to simulated values
   * \param P the error state covariance Matrix to fill
   */
  void setPCore(Eigen::Matrix<double, EKFState_T::nErrorStatesAtCompileTime, EKFState_T::nErrorStatesAtCompileTime>& P){
    enum{
      coreErrorStates = 15 // we might want to calculate this, but on the other hand the values for the matrix later on are anyway hardcoded
    };
    P.setIdentity();
    P *= 0.0001; //set diagonal small covariance for all states

    //now set the core state covariance to the simulated values
    Eigen::Matrix<double, coreErrorStates, coreErrorStates> P_core;
    P_core<<  0.0166, 0.0122,-0.0015, 0.0211, 0.0074, 0.0000, 0.0012,-0.0012, 0.0001,-0.0000, 0.0000,-0.0000,-0.0003,-0.0002,-0.0000,
        0.0129, 0.0508,-0.0020, 0.0179, 0.0432, 0.0006, 0.0020, 0.0004,-0.0002,-0.0000, 0.0000, 0.0000, 0.0003,-0.0002, 0.0000,
        -0.0013,-0.0009, 0.0142,-0.0027, 0.0057, 0.0079, 0.0007, 0.0007, 0.0000,-0.0000,-0.0000, 0.0000,-0.0001,-0.0004,-0.0001,
        0.0210, 0.0162,-0.0026, 0.0437, 0.0083,-0.0017, 0.0016,-0.0021,-0.0014,-0.0000, 0.0000, 0.0000, 0.0003,-0.0001, 0.0000,
        0.0093, 0.0461, 0.0036, 0.0153, 0.0650,-0.0016, 0.0025, 0.0013,-0.0000,-0.0000, 0.0000, 0.0000, 0.0003, 0.0002, 0.0000,
        -0.0000, 0.0005, 0.0080,-0.0019,-0.0021, 0.0130, 0.0001, 0.0001, 0.0000,-0.0000, 0.0000,-0.0000,-0.0003, 0.0001,-0.0001,
        0.0012, 0.0024, 0.0006, 0.0017, 0.0037, 0.0001, 0.0005, 0.0000, 0.0001,-0.0000, 0.0000,-0.0000,-0.0000,-0.0001,-0.0000,
        -0.0011, 0.0008, 0.0007,-0.0023, 0.0019, 0.0001, 0.0000, 0.0005,-0.0001,-0.0000,-0.0000, 0.0000, 0.0001,-0.0001,-0.0000,
        0.0001,-0.0002,-0.0000,-0.0014, 0.0001, 0.0000, 0.0000,-0.0001, 0.0006,-0.0000,-0.0000,-0.0000, 0.0000, 0.0000,-0.0000,
        -0.0000,-0.0000,-0.0000,-0.0000,-0.0000,-0.0000,-0.0000,-0.0000,-0.0000, 0.0000,-0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
        0.0000, 0.0000,-0.0000, 0.0000, 0.0000, 0.0000, 0.0000,-0.0000,-0.0000,-0.0000, 0.0000,-0.0000, 0.0000, 0.0000, 0.0000,
        -0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000,-0.0000, 0.0000,-0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000,-0.0000,
        -0.0003, 0.0003,-0.0001, 0.0003, 0.0003,-0.0003,-0.0000, 0.0001, 0.0000, 0.0000, 0.0000, 0.0000, 0.0010, 0.0000, 0.0000,
        -0.0002,-0.0002,-0.0004,-0.0001, 0.0003, 0.0001,-0.0001,-0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0010, 0.0000,
        -0.0000, 0.0000,-0.0001, 0.0000, 0.0000,-0.0001,-0.0000,-0.0000,-0.0000, 0.0000, 0.0000,-0.0000, 0.0000, 0.0000, 0.0001;
    P.template block<coreErrorStates, coreErrorStates>(0,0) = P_core;
  }

  /**
   * \brief ctor takes a pointer to an object which does the user defined calculations and provides interfaces for initialization etc.
   * \param usercalc the class providing the user defined calculations DO ABSOLUTELY NOT USE THIS REFERENCE INSIDE THIS CTOR!!
   */
  MSF_Core(MSF_SensorManager<EKFState_T>& usercalc);
  ~MSF_Core();


private:
  stateBufferT StateBuffer_; ///<EKF buffer containing pretty much all info needed at time t. sorted by t asc
  measurementBufferT MeasurementBuffer_; ///< EKF Measurements and init values sorted by t asc

  std::queue<boost::shared_ptr<MSF_MeasurementBase<EKFState_T> > > queueFutureMeasurements_; ///< buffer for measurements to apply in future

  double time_P_propagated; ///< last time stamp where we have a valid propagation

  Eigen::Matrix<double, 3, 1> g_; ///< gravity vector

  Eigen::Matrix<double, 3, 3> R_IW_; ///< Rot IMU->World
  Eigen::Matrix<double, 3, 3> R_CI_; ///< Rot Camera->IMU
  Eigen::Matrix<double, 3, 3> R_WV_; ///< Rot World->Vision


  /// vision-world drift watch dog to determine fuzzy tracking

  /**
   * \brief get the index of the best state having no temporal drift at compile time
   */
  enum{
    indexOfStateWithoutTemporalDrift = msf_tmp::IndexOfBestNonTemporalDriftingState<StateSequence_T>::value
  };
  typedef typename msf_tmp::getEnumStateType<StateSequence_T, indexOfStateWithoutTemporalDrift>::value nonDriftingStateType; //returns void type for invalid types

  CheckFuzzyTracking<EKFState_T, nonDriftingStateType> fuzzyTracker_;

  /// enables internal state predictions for log replay
  /**
   * used to determine if internal states get overwritten by the external
   * state prediction (online) or internal state prediction is performed
   * for log replay, when the external prediction is not available.
   */
  bool data_playback_;

  MSF_SensorManager<EKFState_T>& usercalc_; ///< a class which provides methods for customization of several calculations

  enum
  {
    NO_UP, GOOD_UP, FUZZY_UP
  };

  ros::Publisher pubState_; ///< publishes all states of the filter
  sensor_fusion_comm::DoubleArrayStamped msgState_;

  ros::Publisher pubPose_; ///< publishes 6DoF pose output
  geometry_msgs::PoseWithCovarianceStamped msgPose_;

  ros::Publisher pubPoseCrtl_; ///< publishes 6DoF pose including velocity output
  sensor_fusion_comm::ExtState msgPoseCtrl_;

  ros::Publisher pubCorrect_; ///< publishes corrections for external state propagation
  sensor_fusion_comm::ExtEkf msgCorrect_;

  ros::Subscriber subState_; ///< subscriber to external state propagation
  ros::Subscriber subImu_; ///< subscriber to IMU readings

  sensor_fusion_comm::ExtEkf hl_state_buf_; ///< buffer to store external propagation data

  /**
   * \brief propagates the state with given dt
   * \param state_old the state to propagate from
   * \param state_new the state to propagate to
   */
  void propagateState(boost::shared_ptr<EKFState_T>& state_old, boost::shared_ptr<EKFState_T>& state_new);

  /**
   * \brief applies the correction
   * \param delaystate the state to apply the correction on
   * \param correction the correction vector
   * \param fuzzythres the error of the non temporal drifting state allowed before fuzzy tracking will be triggered
   */
  bool applyCorrection(boost::shared_ptr<EKFState_T>& delaystate, ErrorState & correction, double fuzzythres = 0.1);

  /**
   * \brief propagate covariance to a given state in time
   * \param state the state to propagate to from the last propagated time
   */
  void propPToState(boost::shared_ptr<EKFState_T>& state);

  //internal state propagation
  /**
   * \brief This function gets called on incoming imu messages
   * and then performs the state prediction internally.
   * Only use this OR stateCallback by remapping the topics accordingly.
   * \param msg the imu ros message
   * \sa{stateCallback}
   */
  void imuCallback(const sensor_msgs::ImuConstPtr & msg);

  /// external state propagation
  /**
   * \brief This function gets called when state prediction is performed externally,
   * e.g. by asctec_mav_framework. Msg has to be the latest predicted state.
   * Only use this OR imuCallback by remapping the topics accordingly.
   * \param msg the state message from the external propagation
   * \sa{imuCallback}
   */
  void stateCallback(const sensor_fusion_comm::ExtEkfConstPtr & msg);

  /// propagates P by one step to distribute processing load
  void propagatePOneStep();

  ///checks the queue of measurements to be applied in the future
  void handlePendingMeasurements();

};

};// end namespace

#include <msf_core/implementation/msf_core.hpp>

#endif /* MSF_CORE_H_ */
