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

#include "calcQCore.h"
#include <msf_core/eigen_utils.h>
#include <msf_core/msf_sensormanager.hpp>
#include <msf_core/msf_tools.hpp>
#include <msf_core/msf_measurement.hpp>

namespace msf_core
{
template<typename EKFState_T>
MSF_Core<EKFState_T>::MSF_Core(MSF_SensorManager<EKFState_T>& usercalc):usercalc_(usercalc)  //the interface for the user to customize EKF interna, DO ABSOLUTELY NOT USE THIS POINTER INSIDE THIS CTOR!!
{

  Qd_.setZero();

  initialized_ = false;
  predictionMade_ = false;

  g_ << 0, 0, 9.81;

  //TODO later: move all this to the external file and derive from this class. We could by this allow compilation on platforms withour ROS
  // ros stuff
  ros::NodeHandle nh("msf_core");
  ros::NodeHandle pnh("~");

  pubState_ = nh.advertise<sensor_fusion_comm::DoubleArrayStamped> ("state_out", 1);
  pubCorrect_ = nh.advertise<sensor_fusion_comm::ExtEkf> ("correction", 1);
  pubPose_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped> ("pose", 1);
  pubPoseCrtl_ = nh.advertise<sensor_fusion_comm::ExtState> ("ext_state", 1);
  msgState_.data.resize(nStatesAtCompileTime, 0);

  subImu_ = nh.subscribe("imu_state_input", 1, &MSF_Core::imuCallback, this);
  subState_ = nh.subscribe("hl_state_input", 1, &MSF_Core::stateCallback, this);

  msgCorrect_.state.resize(HLI_EKF_STATE_SIZE, 0);
  hl_state_buf_.state.resize(HLI_EKF_STATE_SIZE, 0);


  pnh.param("data_playback", data_playback_, false);


  // buffer for vision failure check
  if(indexOfStateWithoutTemporalDrift != -1){
    nontemporaldrifting_inittimer_ = 1;

    qbuff_ = Eigen::Matrix<double, nBuff_, qbuffRowsAtCompiletime>::Constant(0);
  }
  time_P_propagated = 0;

}

template<typename EKFState_T>
MSF_Core<EKFState_T>::~MSF_Core()
{
}

template<typename EKFState_T>
void MSF_Core<EKFState_T>::initExternalPropagation(boost::shared_ptr<EKFState_T> state) {
  // init external propagation
  msgCorrect_.header.stamp = ros::Time::now();
  msgCorrect_.header.seq = 0;
  msgCorrect_.angular_velocity.x = 0;
  msgCorrect_.angular_velocity.y = 0;
  msgCorrect_.angular_velocity.z = 0;
  msgCorrect_.linear_acceleration.x = 0;
  msgCorrect_.linear_acceleration.y = 0;
  msgCorrect_.linear_acceleration.z = 0;

  msgCorrect_.state.resize(HLI_EKF_STATE_SIZE); //make sure this is correctly sized
  boost::fusion::for_each(
      state->statevars_,
      msf_tmp::CoreStatetoDoubleArray<std::vector<float>, typename EKFState_T::stateVector_T >(msgCorrect_.state)
  );

  msgCorrect_.flag = sensor_fusion_comm::ExtEkf::initialization;
  pubCorrect_.publish(msgCorrect_);
}

template<typename EKFState_T>
void MSF_Core<EKFState_T>::imuCallback(const sensor_msgs::ImuConstPtr & msg)
{
  if(!initialized_)
    return;

  boost::shared_ptr<EKFState_T> lastState = StateBuffer_.getClosestBefore(msg->header.stamp.toSec());

  if(lastState->time_ == -1){
    ROS_WARN_STREAM_THROTTLE(2, "ImuCallback: closest state is invalid\n");
    return; // // early abort // //
  }


  boost::shared_ptr<EKFState_T> currentState(new EKFState_T);

  currentState->time_ = msg->header.stamp.toSec();

  static int seq = 0;

  // get inputs
  currentState->a_m_ << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
  currentState->w_m_ << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;


  // remove acc spikes (TODO: find a cleaner way to do this)
  static Eigen::Matrix<double, 3, 1> last_am = Eigen::Matrix<double, 3, 1>(0, 0, 0);
  if (currentState->a_m_.norm() > 50)
    currentState->a_m_ = last_am;
  else{
    //try to get the state before the current time
    if(lastState->time_ == -1){
      ROS_WARN_STREAM("Accelerometer readings had a spike, but no prior state was in the buffer to take cleaner measurements from");
      return;
    }
    last_am = lastState->a_m_;
  }
  if (!predictionMade_)
  {
    if(lastState->time_ == -1){
      ROS_WARN_STREAM("Wanted to compare prediction time offset to last state, but no prior state was in the buffer to take cleaner measurements from");
      return;
    }
    if (fabs(currentState->time_ - lastState->time_) > 5)
    {
      ROS_WARN_STREAM_THROTTLE(2, "large time-gap re-initializing to last state\n");
      StateBuffer_.updateTime(lastState->time_, currentState->time_);
      time_P_propagated = currentState->time_;
      return; // // early abort // // (if timegap too big)
    }
  }

  if(lastState->time_ == -1){
    ROS_WARN_STREAM("Wanted to propagate state, but no valid prior state could be found in the buffer");
    return;
  }

  propagateState(lastState, currentState);

  propagatePOneStep();

  predictionMade_ = true;

  msgPose_.header.stamp = msg->header.stamp;
  msgPose_.header.seq = msg->header.seq;

  currentState->toPoseMsg(msgPose_);
  pubPose_.publish(msgPose_);

  msgPoseCtrl_.header = msgPose_.header;
  currentState->toExtStateMsg(msgPoseCtrl_);
  pubPoseCrtl_.publish(msgPoseCtrl_);

  StateBuffer_.insert(currentState);


  seq++;
}

template<typename EKFState_T>
void MSF_Core<EKFState_T>::stateCallback(const sensor_fusion_comm::ExtEkfConstPtr & msg)
{

  boost::shared_ptr<EKFState_T> lastState = StateBuffer_.getClosestBefore(msg->header.stamp.toSec());

  if(lastState->time_ == -1)
    return; // // early abort // //


  boost::shared_ptr<EKFState_T> currentState(new EKFState_T);

  currentState->time_ = msg->header.stamp.toSec();

  // get inputs
  currentState->a_m_ << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
  currentState->w_m_ << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;

  // remove acc spikes (TODO: find a cleaner way to do this)
  static Eigen::Matrix<double, 3, 1> last_am = Eigen::Matrix<double, 3, 1>(0, 0, 0);
  if (currentState->a_m_.norm() > 50)
    currentState->a_m_ = last_am;
  else
    last_am = currentState->a_m_;

  if (!predictionMade_)
  {
    if (fabs(currentState->time_ - lastState->time_) > 5)
    {
      ROS_WARN_STREAM_THROTTLE(2, "large time-gap re-initializing to last state\n");
      StateBuffer_.updateTime(lastState->time_, currentState->time_);
      return; // // early abort // // (if timegap too big)
    }
  }

  int32_t flag = msg->flag;
  if (data_playback_)
    flag = sensor_fusion_comm::ExtEkf::ignore_state;

  bool isnumeric = true;
  if (flag == sensor_fusion_comm::ExtEkf::current_state)
    isnumeric = checkForNumeric(&msg->state[0], 10, "before prediction p,v,q");

  isnumeric = checkForNumeric((double*)(&currentState-> template get<msf_updates::p_>()[0]), 3, "before prediction p");

  if (flag == sensor_fusion_comm::ExtEkf::current_state && isnumeric) // state propagation is made externally, so we read the actual state
  {
    currentState-> template get<msf_updates::p_>() = Eigen::Matrix<double, 3, 1>(msg->state[0], msg->state[1], msg->state[2]);
    currentState-> template get<msf_updates::v_>() = Eigen::Matrix<double, 3, 1>(msg->state[3], msg->state[4], msg->state[5]);
    currentState-> template get<msf_updates::q_>() = Eigen::Quaternion<double>(msg->state[6], msg->state[7], msg->state[8], msg->state[9]);
    currentState-> template get<msf_updates::q_>().normalize();

    // zero props:
    //copy non propagation states from last state
    boost::fusion::for_each(
        currentState->statevars_,
        msf_tmp::copyNonPropagationStates<EKFState_T>(*lastState)
    );

    hl_state_buf_ = *msg;
  }
  else if (flag == sensor_fusion_comm::ExtEkf::ignore_state || !isnumeric){ // otherwise let's do the state prop. here
    propagateState(lastState, currentState);
  }

  propagatePOneStep();

  isnumeric = checkForNumeric((double*)(&currentState-> template get<msf_updates::p_>()[0]), 3, "prediction p");
  isnumeric = checkForNumeric((double*)(&currentState->P_(0)), nErrorStatesAtCompileTime * nErrorStatesAtCompileTime, "prediction done P");

  predictionMade_ = true;

  StateBuffer_.insert(currentState);
}


template<typename EKFState_T>
void MSF_Core<EKFState_T>::propagateState(boost::shared_ptr<EKFState_T>& state_old, boost::shared_ptr<EKFState_T>& state_new)
{

  double dt = state_new->time_ - state_old->time_;

  //reset new state to zero
  boost::fusion::for_each(
      state_new->statevars_,
      msf_tmp::resetState()
  );


  // zero props: copy constant for non propagated states
  boost::fusion::for_each(
      state_new->statevars_,
      msf_tmp::copyNonPropagationStates<EKFState_T>(*state_old)
  );

  Eigen::Matrix<double, 3, 1> dv;
  ConstVector3 ew = state_new->w_m_ - state_new-> template get<msf_updates::b_w_>();
  ConstVector3 ewold = state_old->w_m_ - state_old-> template get<msf_updates::b_w_>();
  ConstVector3 ea = state_new->a_m_ - state_new-> template get<msf_updates::b_a_>();
  ConstVector3 eaold = state_old->a_m_ - state_old-> template get<msf_updates::b_a_>();
  ConstMatrix4 Omega = omegaMatJPL(ew);
  ConstMatrix4 OmegaOld = omegaMatJPL(ewold);
  Matrix4 OmegaMean = omegaMatJPL((ew + ewold) / 2);

  // zero order quaternion integration
  //	cur_state.q_ = (Eigen::Matrix<double,4,4>::Identity() + 0.5*Omega*dt)*StateBuffer_[(unsigned char)(idx_state_-1)].q_.coeffs();

  // first order quaternion integration, this is kind of costly and may not add a lot to the quality of propagation...
  int div = 1;
  Matrix4 MatExp;
  MatExp.setIdentity();
  OmegaMean *= 0.5 * dt;
  for (int i = 1; i < 5; i++) //can be made fourth order or less
  {
    div *= i;
    MatExp = MatExp + OmegaMean / div;
    OmegaMean *= OmegaMean;
  }

  // first oder quat integration matrix
  ConstMatrix4 quat_int = MatExp + 1.0 / 48.0 * (Omega * OmegaOld - OmegaOld * Omega) * dt * dt;

  // first oder quaternion integration
  state_new-> template get<msf_updates::q_>().coeffs() = quat_int * state_old-> template get<msf_updates::q_>().coeffs();
  state_new-> template get<msf_updates::q_>().normalize();

  dv = (state_new-> template get<msf_updates::q_>().toRotationMatrix() * ea + state_old-> template get<msf_updates::q_>().toRotationMatrix() * eaold) / 2;
  state_new-> template get<msf_updates::v_>() = state_old-> template get<msf_updates::v_>() + (dv - g_) * dt;
  state_new-> template get<msf_updates::p_>() = state_old-> template get<msf_updates::p_>() + ((state_new-> template get<msf_updates::v_>() + state_old-> template get<msf_updates::v_>()) / 2 * dt);

}

template<typename EKFState_T>
void MSF_Core<EKFState_T>::propagatePOneStep(){
  //also propagate the covariance one step further, to distribute the processing load over time
  typename stateBufferT::iterator_T stateIteratorPLastPropagated = StateBuffer_.getIteratorAtValue(time_P_propagated);
  typename stateBufferT::iterator_T stateIteratorPLastPropagatedNext = stateIteratorPLastPropagated;
  ++stateIteratorPLastPropagatedNext;
  if(stateIteratorPLastPropagatedNext != StateBuffer_.getIteratorEnd()){ //might happen if there is a measurement in the future
    predictProcessCovariance(stateIteratorPLastPropagated->second, stateIteratorPLastPropagatedNext->second);
    checkForNumeric((double*)(&stateIteratorPLastPropagatedNext->second-> template get<msf_updates::p_>()(0)), 3, "prediction p");
  }
}

template<typename EKFState_T>
void MSF_Core<EKFState_T>::predictProcessCovariance(boost::shared_ptr<EKFState_T>& state_old, boost::shared_ptr<EKFState_T>& state_new)
{

  //TODO: later: find out whether we have to linearize around the current or the last state

  double dt = state_new->time_ - state_old->time_;

  // noises
  ConstVector3 nav = Vector3::Constant(usercalc_.getParam_noise_acc() /* / sqrt(dt) */);
  ConstVector3 nbav = Vector3::Constant(usercalc_.getParam_noise_accbias() /* * sqrt(dt) */);

  ConstVector3 nwv = Vector3::Constant(usercalc_.getParam_noise_gyr() /* / sqrt(dt) */);
  ConstVector3 nbwv = Vector3::Constant(usercalc_.getParam_noise_gyrbias() /* * sqrt(dt) */);

  // bias corrected IMU readings
  ConstVector3 ew = state_new->w_m_ - state_new-> template get<msf_updates::b_w_>();
  ConstVector3 ea = state_new->a_m_ - state_new-> template get<msf_updates::b_a_>();

  ConstMatrix3 a_sk = skew(ea);
  ConstMatrix3 w_sk = skew(ew);
  ConstMatrix3 eye3 = Eigen::Matrix<double, 3, 3>::Identity();

  ConstMatrix3 C_eq = state_new-> template get<msf_updates::q_>().toRotationMatrix();

  const double dt_p2_2 = dt * dt * 0.5; // dt^2 / 2
  const double dt_p3_6 = dt_p2_2 * dt / 3.0; // dt^3 / 6
  const double dt_p4_24 = dt_p3_6 * dt * 0.25; // dt^4 / 24
  const double dt_p5_120 = dt_p4_24 * dt * 0.2; // dt^5 / 120

  ConstMatrix3 Ca3 = C_eq * a_sk;
  ConstMatrix3 A = Ca3 * (-dt_p2_2 * eye3 + dt_p3_6 * w_sk - dt_p4_24 * w_sk * w_sk);
  ConstMatrix3 B = Ca3 * (dt_p3_6 * eye3 - dt_p4_24 * w_sk + dt_p5_120 * w_sk * w_sk);
  ConstMatrix3 D = -A;
  ConstMatrix3 E = eye3 - dt * w_sk + dt_p2_2 * w_sk * w_sk;
  ConstMatrix3 F = -dt * eye3 + dt_p2_2 * w_sk - dt_p3_6 * (w_sk * w_sk);
  ConstMatrix3 C = Ca3 * F;

  // discrete error state propagation Matrix Fd according to:
  // Stephan Weiss and Roland Siegwart.
  // Real-Time Metric State Estimation for Modular Vision-Inertial Systems.
  // IEEE International Conference on Robotics and Automation. Shanghai, China, 2011
  Fd_.setIdentity();
  Fd_. template block<3, 3> (0, 3) = dt * eye3;
  Fd_. template block<3, 3> (0, 6) = A;
  Fd_. template block<3, 3> (0, 9) = B;
  Fd_. template block<3, 3> (0, 12) = -C_eq * dt_p2_2;

  Fd_. template block<3, 3> (3, 6) = C;
  Fd_. template block<3, 3> (3, 9) = D;
  Fd_. template block<3, 3> (3, 12) = -C_eq * dt;

  Fd_. template block<3, 3> (6, 6) = E;
  Fd_. template block<3, 3> (6, 9) = F;

  calc_QCore(dt, state_new-> template get<msf_updates::q_>(), ew, ea, nav, nbav, nwv, nbwv, Qd_);

  //call user Q calc to fill in the blocks of auxiliary states
  usercalc_.calculateQAuxiliaryStates(*state_new, dt);

  //now copy the userdefined blocks to Qd
  boost::fusion::for_each(
      state_new->statevars_,
      msf_tmp::copyQBlocksFromAuxiliaryStatesToQ<typename EKFState_T::stateVector_T>(Qd_)
  );

  //TODO later: optimize here multiplication of F blockwise, using the fact that aux states have no entries outside their block
  state_new->P_ = Fd_ * state_old->P_ * Fd_.transpose() + Qd_;

  //set time for best cov prop to now
  time_P_propagated = state_new->time_;

}

template<typename EKFState_T>
void MSF_Core<EKFState_T>::init(boost::shared_ptr<MSF_MeasurementBase<EKFState_T> > measurement){
  MeasurementBuffer_.clear();
  StateBuffer_.clear();

  //push one state to the buffer to apply the init on
  boost::shared_ptr<EKFState_T> state(new EKFState_T);
  state->time_ = 0;

  //reset new state to zero
  boost::fusion::for_each(
      state->statevars_,
      msf_tmp::resetState()
  );

  //set intialial covariance for core states
  setPCore(state->P_);

  //apply init measurement, where the user can provide additional values for P
  measurement->apply(state, *this);

  StateBuffer_.insert(state);
  time_P_propagated = 0; //will be set upon first IMU message

  initialized_ = true;
  predictionMade_ = false;
}

template<typename EKFState_T>
void MSF_Core<EKFState_T>::addMeasurement(boost::shared_ptr<MSF_MeasurementBase<EKFState_T> > measurement){

  if(!initialized_ || !predictionMade_)
    return;

  typename measurementBufferT::iterator_T it_meas = MeasurementBuffer_.insert(measurement);
  typename measurementBufferT::iterator_T it_meas_end = MeasurementBuffer_.getIteratorEnd();
  typename stateBufferT::iterator_T it_curr;

  bool appliedOne = false;

  for( ; it_meas != it_meas_end; ++it_meas){

    boost::shared_ptr<EKFState_T> state = getClosestState(it_meas->second->time_); //propagates covariance to state

    if(state->time_ <= 0){
      ROS_ERROR_STREAM_THROTTLE(1,"getClosestState returned an invalid state");
      continue;
    }

    it_meas->second->apply(state, *this); //calls back core::applyCorrection(), which sets time_P_propagated to meas time

    //make sure to propagate to next measurement or up to now if no more measurements
    it_curr = StateBuffer_.getIteratorAtValue(state); //propagate from current state

    typename stateBufferT::iterator_T it_end;
    typename measurementBufferT::iterator_T it_nextmeas = it_meas;
    ++it_nextmeas; //the next measurement in the list

    if(it_nextmeas == it_meas_end){ //that was the last measurement, so propagate state to now
      it_end = StateBuffer_.getIteratorEnd();
    }else{
      it_end = StateBuffer_.getIteratorClosest(it_nextmeas->second->time_); //get state closest to next measurement TODO: do we want to also respect the delay here. Otherwise we might propagate a bit to much
      if(it_end == StateBuffer_.getIteratorEnd()){
        ROS_ERROR_STREAM("Wanted to get a state close to the next measurement, but got end of list");
        --it_end;
      }
    }

    typename stateBufferT::iterator_T it_next = it_curr;
    ++it_next;

    for( ; it_curr != it_end && it_next != it_end; ++it_curr, ++it_next){ //propagate to selected state
      propagateState(it_curr->second, it_next->second);
    }

    appliedOne = true;
  }

  if(!appliedOne)
    return;

  //now publish the best current estimate
  static int seq_m = 0;


  // publish correction for external propagation
  msgCorrect_.header.stamp = ros::Time::now();
  msgCorrect_.header.seq = seq_m;
  msgCorrect_.angular_velocity.x = 0;
  msgCorrect_.angular_velocity.y = 0;
  msgCorrect_.angular_velocity.z = 0;
  msgCorrect_.linear_acceleration.x = 0;
  msgCorrect_.linear_acceleration.y = 0;
  msgCorrect_.linear_acceleration.z = 0;
  boost::shared_ptr<EKFState_T>& latestState = StateBuffer_.getLast();

  // prevent junk being sent to the external state propagation when data playback is (accidentally) on
  if(data_playback_){
    for(int i=0; i<HLI_EKF_STATE_SIZE; ++i){
      msgCorrect_.state[i] = 0;
    }
    msgCorrect_.state[6] = 1;
    msgCorrect_.flag = sensor_fusion_comm::ExtEkf::initialization;
  }
  else{
    msgCorrect_.state[0] = latestState-> template get<msf_updates::p_>()[0] - hl_state_buf_.state[0];
    msgCorrect_.state[1] = latestState-> template get<msf_updates::p_>()[1] - hl_state_buf_.state[1];
    msgCorrect_.state[2] = latestState-> template get<msf_updates::p_>()[2] - hl_state_buf_.state[2];
    msgCorrect_.state[3] = latestState-> template get<msf_updates::v_>()[0] - hl_state_buf_.state[3];
    msgCorrect_.state[4] = latestState-> template get<msf_updates::v_>()[1] - hl_state_buf_.state[4];
    msgCorrect_.state[5] = latestState-> template get<msf_updates::v_>()[2] - hl_state_buf_.state[5];

    Eigen::Quaterniond hl_q(hl_state_buf_.state[6], hl_state_buf_.state[7], hl_state_buf_.state[8], hl_state_buf_.state[9]);
    Eigen::Quaterniond qbuff_q = hl_q.inverse() * latestState-> template get<msf_updates::q_>();
    msgCorrect_.state[6] = qbuff_q.w();
    msgCorrect_.state[7] = qbuff_q.x();
    msgCorrect_.state[8] = qbuff_q.y();
    msgCorrect_.state[9] = qbuff_q.z();

    msgCorrect_.state[10] = latestState-> template get<msf_updates::b_w_>()[0] - hl_state_buf_.state[10];
    msgCorrect_.state[11] = latestState-> template get<msf_updates::b_w_>()[1] - hl_state_buf_.state[11];
    msgCorrect_.state[12] = latestState-> template get<msf_updates::b_w_>()[2] - hl_state_buf_.state[12];
    msgCorrect_.state[13] = latestState-> template get<msf_updates::b_a_>()[0] - hl_state_buf_.state[13];
    msgCorrect_.state[14] = latestState-> template get<msf_updates::b_a_>()[1] - hl_state_buf_.state[14];
    msgCorrect_.state[15] = latestState-> template get<msf_updates::b_a_>()[2] - hl_state_buf_.state[15];

    msgCorrect_.flag = sensor_fusion_comm::ExtEkf::state_correction;
  }

  pubCorrect_.publish(msgCorrect_);

  // publish state
  msgState_.header = msgCorrect_.header;
  latestState->toFullStateMsg(msgState_);
  pubState_.publish(msgState_);
  seq_m++;
}

template<typename EKFState_T>
boost::shared_ptr<EKFState_T> MSF_Core<EKFState_T>::getClosestState(double tstamp)
{

  //sweiss{
  // we subtracted one too much before.... :)
  //}

  double timenow = tstamp; // - delay - usercalc_.getParam_delay();

  typename stateBufferT::iterator_T it = StateBuffer_.getIteratorClosest(timenow);

  //TODO: remove this relict from the state_idx days or find out why we need it and document here.
  //it seems this is needed when there was only one imu reading before the first measurement arrives{
  typename stateBufferT::iterator_T itbeg = StateBuffer_.getIteratorBegin();
  static bool started = false;
  if (itbeg == it && !started){
    ++it;
  }
  started = true;
  //}

  boost::shared_ptr<EKFState_T>& closestState = it->second;

  if (closestState->time_ == -1){
    ROS_WARN_STREAM("Requested closest state to "<<timenow<<" but there was no suitable state in the map");
    return StateBuffer_.getInvalid(); // // early abort // //  not enough predictions made yet to apply measurement (too far in past)
  }
  propPToState(closestState); // catch up with covariance propagation if necessary

  CleanUpBuffers(); //remove very old states and measurements from the buffers

  return closestState;
}

template<typename EKFState_T>
void MSF_Core<EKFState_T>::propPToState(boost::shared_ptr<EKFState_T>& state)
{
  // propagate cov matrix until the current states time
  typename stateBufferT::iterator_T it = StateBuffer_.getIteratorAtValue(time_P_propagated);
  typename stateBufferT::iterator_T itMinus = it;
  ++it;
  //until we reached the current state or the end of the state list
  for( ; it != StateBuffer_.getIteratorEnd() && it->second->time_ <= state->time_ ; ++it, ++itMinus){
    predictProcessCovariance(itMinus->second, it->second);
  }
}

template<typename EKFState_T>
bool MSF_Core<EKFState_T>::applyCorrection(boost::shared_ptr<EKFState_T>& delaystate, ErrorState & correction, double fuzzythres)
{

  if(!initialized_ || !predictionMade_)
    return false;

  //give the user the possibility to fix some states
  usercalc_.augmentCorrectionVector(correction);

  //now augment core states
  if (usercalc_.getParam_fixed_bias())
  {
    typedef typename msf_tmp::getEnumStateType<typename EKFState_T::stateVector_T, msf_updates::b_a_>::value b_a_type;
    typedef typename msf_tmp::getEnumStateType<typename EKFState_T::stateVector_T, msf_updates::b_w_>::value b_w_type;

    enum{
      indexOfState_b_a = msf_tmp::getStartIndex<typename EKFState_T::stateVector_T, b_a_type, msf_tmp::CorrectionStateLengthForType>::value,
      indexOfState_b_w = msf_tmp::getStartIndex<typename EKFState_T::stateVector_T, b_w_type, msf_tmp::CorrectionStateLengthForType>::value
    };

    BOOST_STATIC_ASSERT_MSG(static_cast<int>(indexOfState_b_w)==9, "The index of the state b_w in the correction vector differs from the expected value");
    BOOST_STATIC_ASSERT_MSG(static_cast<int>(indexOfState_b_a)==12, "The index of the state b_a in the correction vector differs from the expected value");

    for(int i = 0 ; i < msf_tmp::StripConstReference<b_a_type>::result_t::sizeInCorrection_ ; ++i){
      correction(indexOfState_b_a + i) = 0; //acc bias x,y,z
    }
    for(int i = 0 ; i < msf_tmp::StripConstReference<b_w_type>::result_t::sizeInCorrection_ ; ++i){
      correction(indexOfState_b_w + i) = 0; //gyro bias x,y,z
    }
  }

  // state update:
  // TODO: sweiss what to do with attitude? augment measurement noise?
  // store old values in case of fuzzy tracking
  EKFState_T buffstate = *delaystate;

  //call correction function for every state
  delaystate->correct(correction);

  //allow the user to sanity check the new state
  usercalc_.sanityCheckCorrection(*delaystate, buffstate, correction);

  //TODO: move the whole fuzzy tracking to a templated function, which is specialized for void, Matrix, Quaternion

  if(indexOfStateWithoutTemporalDrift != -1){ //is there a state without temporal drift?

    //for now make sure the non drifting state is a quaternion.

    const bool isquaternion = msf_tmp::isQuaternionType<typename msf_tmp::StripConstReference<nonDriftingStateType>::result_t >::value;
    BOOST_STATIC_ASSERT_MSG(isquaternion, "Assumed that the non drifting state is a Quaternion, "
                            "which is not the case for the currently defined state vector. If you want to use an euclidean state, please first adapt qbuff and the error detection routines");


    // update qbuff_ and check for fuzzy tracking
    if (nontemporaldrifting_inittimer_ > nBuff_)
    {
      // should be unit quaternion if no error
      Eigen::Quaternion<double> errq = const_cast<const EKFState_T&>(*delaystate). template get<indexOfStateWithoutTemporalDrift>().conjugate() *
          Eigen::Quaternion<double>(
              getMedian(qbuff_. template block<nBuff_, 1> (0, 3)),
              getMedian(qbuff_. template block<nBuff_, 1> (0, 0)),
              getMedian(qbuff_. template block<nBuff_, 1> (0, 1)),
              getMedian(qbuff_. template block<nBuff_, 1> (0, 2))
          );

      double fuzzythres = usercalc_.getParam_fuzzythres();

      if (std::max(errq.vec().maxCoeff(), -errq.vec().minCoeff()) / fabs(errq.w()) * 2 > fuzzythres) // fuzzy tracking (small angle approx)
      {
        ROS_WARN_STREAM_THROTTLE(1,"fuzzy tracking triggered: " << std::max(errq.vec().maxCoeff(), -errq.vec().minCoeff())/fabs(errq.w())*2 << " limit: " << fuzzythres <<"\n");

        //copy the non propagation states back from the buffer
        boost::fusion::for_each(
            delaystate->statevars_,
            msf_tmp::copyNonPropagationStates<EKFState_T>(buffstate)
        );

        BOOST_STATIC_ASSERT_MSG(static_cast<int>(EKFState_T::nPropagatedCoreErrorStatesAtCompileTime) == 9, "Assumed that nPropagatedCoreStates == 9, which is not the case");
        BOOST_STATIC_ASSERT_MSG(static_cast<int>(EKFState_T::nErrorStatesAtCompileTime) -
                                static_cast<int>(EKFState_T::nPropagatedCoreErrorStatesAtCompileTime) == 16, "Assumed that nErrorStatesAtCompileTime-nPropagatedCoreStates == 16, which is not the case");

      }
      else // if tracking ok: update mean and 3sigma of past N q_vw's
      {
        qbuff_. template block<1, 4> (nontemporaldrifting_inittimer_ - nBuff_ - 1, 0) = Eigen::Matrix<double, 1, 4>(const_cast<const EKFState_T&>(*delaystate). template get<indexOfStateWithoutTemporalDrift>().coeffs());
        nontemporaldrifting_inittimer_ = (nontemporaldrifting_inittimer_) % nBuff_ + nBuff_ + 1;
      }
    }
    else // at beginning get mean and 3sigma of past N q_vw's
    {
      qbuff_. template block<1, 4> (nontemporaldrifting_inittimer_ - 1, 0) = Eigen::Matrix<double, 1, 4>(const_cast<const EKFState_T&>(*delaystate). template get<indexOfStateWithoutTemporalDrift>().coeffs());
      nontemporaldrifting_inittimer_++;
    }
  } //end fuzzy tracking

  //no publishing and propagation here, because this might not be the last update we have to apply

  checkForNumeric(&correction[0], HLI_EKF_STATE_SIZE, "update");

  time_P_propagated = delaystate->time_; //set time latest propagated, we need to repropagate at least from here

  return 1;
}




}; // end namespace
