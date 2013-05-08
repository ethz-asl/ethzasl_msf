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
#include <msf_core/msf_sensormanager.h>
#include <msf_core/msf_tools.h>
#include <msf_core/msf_measurement.h>
#include <chrono>
#include <thread>
#include <sm/timing/Timer.hpp>
#include <deque>
#include <numeric>
#include <sensor_msgs/image_encodings.h>
#include <msf_core/falsecolor.h>

namespace msf_core
{
template<typename EKFState_T>
MSF_Core<EKFState_T>::MSF_Core(MSF_SensorManager<EKFState_T>& usercalc):usercalc_(usercalc)  //the interface for the user to customize EKF interna, DO ABSOLUTELY NOT USE THIS POINTER INSIDE THIS CTOR!!
{
  std::setprecision(NUMERIC_PREC); //set the output precision for numeric values

  initialized_ = false;
  predictionMade_ = false;
  isfuzzyState_ = false;

  //  g_ << 0, 0, 9.80834; //at 47.37 lat

  // TODO Matlab back
  g_ << 0, 0, 9.81;
  //////////////////////

  //TODO later: move all this to the external file and derive from this class. We could by this allow compilation on platforms withour ROS
  ros::NodeHandle nh("msf_core");
  ros::NodeHandle pnh("~");

  pubState_ = nh.advertise<sensor_fusion_comm::DoubleArrayStamped> ("state_out", 100);
  pubCorrect_ = nh.advertise<sensor_fusion_comm::ExtEkf> ("correction", 1);
  pubPose_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped> ("pose", 100);
  pubPoseAfterUpdate_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped> ("pose_after_update", 100);
  pubPoseCrtl_ = nh.advertise<sensor_fusion_comm::ExtState> ("ext_state", 1);
  msgState_.data.resize(nStatesAtCompileTime, 0);

#ifdef  WITHCOVIMAGE
  pubCov_ = nh.advertise<sensor_msgs::Image>("covariance_img", 1);
#endif

  subImu_ = nh.subscribe("imu_state_input", 10, &MSF_Core::imuCallback, this);
  subImuCustom_ = nh.subscribe("imu_state_input_asctec", 10, &MSF_Core::imuCallback_asctec, this);
  subState_ = nh.subscribe("hl_state_input", 10, &MSF_Core::stateCallback, this);

  msgCorrect_.state.resize(HLI_EKF_STATE_SIZE, 0);
  hl_state_buf_.state.resize(HLI_EKF_STATE_SIZE, 0);


  pnh.param("data_playback", data_playback_, false);

  time_P_propagated = 0;

}

template<typename EKFState_T>
MSF_Core<EKFState_T>::~MSF_Core()
{
}

template<typename EKFState_T>
void MSF_Core<EKFState_T>::initExternalPropagation(boost::shared_ptr<EKFState_T> state) {
  // init external propagation
  msgCorrect_.header.stamp = ros::Time(state->time);
  msgCorrect_.header.seq = 0;
  msgCorrect_.angular_velocity.x = 0;
  msgCorrect_.angular_velocity.y = 0;
  msgCorrect_.angular_velocity.z = 0;
  msgCorrect_.linear_acceleration.x = 0;
  msgCorrect_.linear_acceleration.y = 0;
  msgCorrect_.linear_acceleration.z = 0;

  msgCorrect_.state.resize(HLI_EKF_STATE_SIZE);
  boost::fusion::for_each(
      state->statevars,
      msf_tmp::CoreStatetoDoubleArray<std::vector<float>, StateSequence_T >(msgCorrect_.state)
  );

  msgCorrect_.flag = sensor_fusion_comm::ExtEkf::initialization;
  pubCorrect_.publish(msgCorrect_);
}

template<typename EKFState_T>
void MSF_Core<EKFState_T>::publishCovImage(boost::shared_ptr<EKFState_T> stateptr) const {

#ifdef  WITHCOVIMAGE
  if(!pubCov_.getNumSubscribers())
    return;

  const EKFState_T& state = *stateptr;

  static int imgseq = 0;

  enum{
    blocksize = 10, //size of cov blocks in pixels
    imgrows = EKFState_T::nErrorStatesAtCompileTime * blocksize + EKFState_T::nStateVarsAtCompileTime - 1
  };

  cv::Mat colorimg(imgrows, imgrows, CV_8UC3);
  colorimg.setTo(0);

  std::vector<std::tuple<int, int, int> > enumsandindices;
  stateptr->calculateIndicesInErrorState(enumsandindices);

  //smooth min max values of P
  static struct smoothP{
    std::deque<double> dmin;
    std::deque<double> dmax;
  } smoother;

  smoother.dmin.push_back(state.P.minCoeff());
  smoother.dmax.push_back(state.P.maxCoeff());

  size_t filtsize = 50;
  if(smoother.dmin.size() > filtsize)
    smoother.dmin.pop_front();

  if(smoother.dmax.size() > filtsize)
    smoother.dmax.pop_front();

  double min = std::accumulate(smoother.dmin.begin(), smoother.dmin.end(), 0.0) / smoother.dmin.size();
  double max = std::accumulate(smoother.dmax.begin(), smoother.dmax.end(), 0.0) / smoother.dmax.size();

  static palette pal = GetPalette(palette::False_color_palette4);

  //draw the blocks for covs of the state variables
  for(size_t i = 0; i < enumsandindices.size() ; ++i){
    for(size_t j = 0; j < enumsandindices.size() ; ++j){
      int lengthincorrectionrows = std::get<2>(enumsandindices.at(i));
      int lengthincorrectioncols = std::get<2>(enumsandindices.at(j));
      //print all entries for this state combination
      for(int rowidx = 0;rowidx<lengthincorrectionrows;++rowidx){
        for(int colidx = 0;colidx<lengthincorrectioncols;++colidx){

          int startrow = std::get<0>(enumsandindices.at(i)) + (std::get<1>(enumsandindices.at(i)) + rowidx) * blocksize;
          int startcol = std::get<0>(enumsandindices.at(j)) + (std::get<1>(enumsandindices.at(j)) + colidx) * blocksize;

          double value = state.P(std::get<1>(enumsandindices.at(i)) + rowidx, std::get<1>(enumsandindices.at(j)) + colidx);

          int brightness = (value - min) / (max - min) * 255.;
          brightness = brightness > 255 ? 255 : brightness < 0 ? 0 : brightness; //clamp

          cv::rectangle(colorimg, cv::Point(startrow, startcol),cv::Point(startrow + blocksize, startcol + blocksize),
                        CV_RGB(pal.colors[brightness].rgbRed, pal.colors[brightness].rgbGreen, pal.colors[brightness].rgbBlue), CV_FILLED);

        }
      }
    }
  }

  //draw the lines between the state variables
  for(size_t i = 1; i < enumsandindices.size() ; ++i){

    int startrow = std::get<0>(enumsandindices.at(i)) + std::get<1>(enumsandindices.at(i)) * blocksize;

    cv::line(colorimg,cv::Point(startrow, 0),cv::Point(startrow, imgrows),CV_RGB(227,176,55));
    cv::line(colorimg,cv::Point(0, startrow),cv::Point(imgrows, startrow),CV_RGB(227,176,55));

  }

  sensor_msgs::ImagePtr msgimg_(new sensor_msgs::Image);
  msgimg_->data.resize(colorimg.cols*colorimg.rows*3);
  msgimg_->header.stamp = ros::Time(state.time);
  msgimg_->header.seq = ++imgseq;
  msgimg_->width = colorimg.cols;
  msgimg_->height = colorimg.rows;
  msgimg_->encoding = sensor_msgs::image_encodings::BGR8;
  msgimg_->step =colorimg.cols*3;
  msgimg_->is_bigendian = 0;
  memcpy(&msgimg_->data[0],&colorimg.data[0],sizeof(char)*msgimg_->data.size());

  pubCov_.publish(msgimg_);

  //  ROS_WARN_STREAM("P=["<<state.P<<"];");
#else
  ROS_INFO_STREAM_ONCE("The function to publish a covariance image was called, but the function is disabled at compile time.");
  UNUSED(stateptr)
#endif

}

template<typename EKFState_T>
void MSF_Core<EKFState_T>::setPCore(Eigen::Matrix<double, EKFState_T::nErrorStatesAtCompileTime, EKFState_T::nErrorStatesAtCompileTime>& P){
  enum{
    coreErrorStates = 15 // we might want to calculate this, but on the other hand the values for the matrix later on are anyway hardcoded
  };
  P.setIdentity();
  P *= 0.0001; //set diagonal small covariance for all states

  //now set the core state covariance to the simulated values
  Eigen::Matrix<double, coreErrorStates, coreErrorStates> P_core;
//    P_core<<  0.0166, 0.0122,-0.0015, 0.0211, 0.0074, 0.0000, 0.0012,-0.0012, 0.0001,-0.0000, 0.0000,-0.0000,-0.0003,-0.0002,-0.0000,
//        0.0129, 0.0508,-0.0020, 0.0179, 0.0432, 0.0006, 0.0020, 0.0004,-0.0002,-0.0000, 0.0000, 0.0000, 0.0003,-0.0002, 0.0000,
//        -0.0013,-0.0009, 0.0142,-0.0027, 0.0057, 0.0079, 0.0007, 0.0007, 0.0000,-0.0000,-0.0000, 0.0000,-0.0001,-0.0004,-0.0001,
//        0.0210, 0.0162,-0.0026, 0.0437, 0.0083,-0.0017, 0.0016,-0.0021,-0.0014,-0.0000, 0.0000, 0.0000, 0.0003,-0.0001, 0.0000,
//        0.0093, 0.0461, 0.0036, 0.0153, 0.0650,-0.0016, 0.0025, 0.0013,-0.0000,-0.0000, 0.0000, 0.0000, 0.0003, 0.0002, 0.0000,
//        -0.0000, 0.0005, 0.0080,-0.0019,-0.0021, 0.0130, 0.0001, 0.0001, 0.0000,-0.0000, 0.0000,-0.0000,-0.0003, 0.0001,-0.0001,
//        0.0012, 0.0024, 0.0006, 0.0017, 0.0037, 0.0001, 0.0005, 0.0000, 0.0001,-0.0000, 0.0000,-0.0000,-0.0000,-0.0001,-0.0000,
//        -0.0011, 0.0008, 0.0007,-0.0023, 0.0019, 0.0001, 0.0000, 0.0005,-0.0001,-0.0000,-0.0000, 0.0000, 0.0001,-0.0001,-0.0000,
//        0.0001,-0.0002,-0.0000,-0.0014, 0.0001, 0.0000, 0.0000,-0.0001, 0.0006,-0.0000,-0.0000,-0.0000, 0.0000, 0.0000,-0.0000,
//        -0.0000,-0.0000,-0.0000,-0.0000,-0.0000,-0.0000,-0.0000,-0.0000,-0.0000, 0.0000,-0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
//        0.0000, 0.0000,-0.0000, 0.0000, 0.0000, 0.0000, 0.0000,-0.0000,-0.0000,-0.0000, 0.0000,-0.0000, 0.0000, 0.0000, 0.0000,
//        -0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000,-0.0000, 0.0000,-0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000,-0.0000,
//        -0.0003, 0.0003,-0.0001, 0.0003, 0.0003,-0.0003,-0.0000, 0.0001, 0.0000, 0.0000, 0.0000, 0.0000, 0.0010, 0.0000, 0.0000,
//        -0.0002,-0.0002,-0.0004,-0.0001, 0.0003, 0.0001,-0.0001,-0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0010, 0.0000,
//        -0.0000, 0.0000,-0.0001, 0.0000, 0.0000,-0.0001,-0.0000,-0.0000,-0.0000, 0.0000, 0.0000,-0.0000, 0.0000, 0.0000, 0.0001;


  P_core<< 3.00086,  7.51337e-05,  1.66385e-05,  8.93782e-06, 1.53845e-06, -1.2653e-05, -6.61888e-07, -1.37912e-05,  0.000131836, -6.04721e-08,  2.76981e-07, -3.79139e-06,   0.00016278,  1.16048e-05,  1.34879e-06,
   7.51337e-05,3.00076,  3.99831e-05, -1.26983e-05,  2.26705e-05 , 5.28182e-06, -8.98194e-06,  7.19556e-06,   0.00024316  ,3.10009e-07  , 9.5851e-08, -8.00355e-06, -1.76053e-05, -4.03244e-05,  5.57302e-07,
   1.66384e-05,  3.99831e-05, 0.000874768, -2.2499e-05 ,-2.84821e-05, 9.73556e-05 , 2.04936e-05, 9.31698e-06, 2.1803e-05, -3.06216e-08, -1.44031e-07, -6.39018e-07 , -4.0063e-05 , 0.000191473,  2.87644e-06,
   8.93829e-06, -1.26964e-05, -2.25004e-05,  2.37453e-05, -1.04012e-06 ,-5.40794e-06, -5.31478e-07, -3.76391e-06, -3.98025e-05, -5.87104e-09,  5.44963e-08,  1.23138e-06,  1.23169e-05, -1.34348e-05, -1.32695e-07,
   1.53704e-06,   2.2671e-05, -2.84805e-05, -1.16443e-06,  2.63155e-05, -9.95426e-07, -3.15046e-06,  1.63826e-06,  2.84765e-05,  5.66775e-08,  1.38911e-08,  -9.0897e-07, -1.29378e-05, -1.10091e-05, -4.79721e-08,
   -1.2653e-05,  5.28182e-06,  9.73556e-05, -5.40759e-06, -9.95318e-07,  3.28836e-05,  2.49675e-06,  8.00741e-06,  9.70208e-06,   4.4046e-09, -1.32115e-08,  -2.8648e-07, -7.23793e-05,  2.67769e-05,  8.33117e-07,
  -6.62245e-07,  -8.9832e-06,  2.04947e-05, -5.31688e-07, -3.23185e-06,  2.49698e-06,  8.43701e-06,  -1.4539e-06, -1.89459e-05, -1.13307e-08, -1.51975e-08,  5.81262e-07,  1.31976e-05,   7.5001e-05, -4.88314e-08,
  -1.37913e-05,   7.1956e-06,  9.31721e-06, -3.77545e-06,  1.63842e-06,  8.00743e-06, -1.45398e-06,  7.41575e-06,  2.43303e-05,  6.97444e-09,  -4.8326e-09, -7.52004e-07, -6.53142e-05, -8.47091e-06,   7.6716e-08,
   0.000131836,   0.00024316,  2.18031e-05, -3.98031e-05,  2.84757e-05,   9.7021e-06, -1.89454e-05,  2.43302e-05,  0.000731082,  4.89429e-08,  9.29108e-08, -2.31437e-05, -7.87083e-05, -5.39542e-05,  4.66545e-07,
  -6.04721e-08,  3.10009e-07, -3.06222e-08, -5.86548e-09,  5.66801e-08,  4.40452e-09, -1.13349e-08,  6.97452e-09,  4.89427e-08,  5.34985e-09,  8.64308e-11, -2.52926e-09, -6.71258e-08, -1.12465e-08,  1.95952e-10,
   2.76981e-07,  9.58509e-08,  -1.4403e-07,  5.44966e-08,  1.38855e-08, -1.32114e-08, -1.51976e-08, -4.83317e-09,  9.29106e-08,  8.64324e-11,  5.35169e-09, -4.96914e-09, -1.83667e-08, -1.34507e-07, -1.76329e-10,
  -3.79139e-06, -8.00355e-06, -6.39022e-07,   1.2314e-06, -9.08938e-07, -2.86481e-07,  5.81245e-07, -7.52001e-07, -2.31437e-05, -2.52927e-09, -4.96915e-09,  7.57833e-07,  2.34049e-06,   1.5266e-06, -1.27473e-08,
    0.00016278, -1.76053e-05, -4.00629e-05,  1.23157e-05, -1.29381e-05, -7.23793e-05,  1.31984e-05, -6.53142e-05, -7.87082e-05, -6.71258e-08, -1.83663e-08,  2.34048e-06,  0.000628949,   9.9532e-05, -5.55973e-07,
   1.16049e-05, -4.03244e-05,  0.000191472, -1.34348e-05, -1.10082e-05,  2.67769e-05,  7.50009e-05, -8.47082e-06, -5.39542e-05, -1.12468e-08, -1.34507e-07,   1.5266e-06,   9.9532e-05,  0.000722253, -4.20019e-07,
   1.34879e-06,  5.57302e-07,  2.87644e-06, -1.32689e-07, -4.79779e-08,  8.33117e-07,  -4.8836e-08,  7.67151e-08,  4.66545e-07,  1.95954e-10,  -1.7633e-10, -1.27473e-08, -5.55973e-07, -4.20019e-07,  1.67777e-07;


//  //TODO Matlab back
//  P_core.setZero();
//  P_core(0,0) = 3;
//  P_core(1,1) = 3;
//  P_core(2,2) = 3;
//  P_core(3,3) = 2;
//  P_core(4,4) = 2;
//  P_core(5,5) = 2;
//  P_core(6,6) = 0.04;
//  P_core(7,7) = 0.04;
//  P_core(8,8) = 0.04;
//  P_core(9,9) = 0.015;
//  P_core(10,10) = 0.015;
//  P_core(11,11) = 0.015;
//  P_core(12,12) = 0.163;
//  P_core(13,13) = 0.163;
//  P_core(14,14) = 0.163;
//  ///////////////////////////////


  P_core = 0.5 * (P_core + P_core.transpose());
  P.template block<coreErrorStates, coreErrorStates>(0,0) = P_core;


}

template<typename EKFState_T>
void MSF_Core<EKFState_T>::imuCallback_asctec(const  asctec_hl_comm::mav_imuConstPtr & msg)
{

  msf_core::Vector3 linacc;
  linacc << msg->acceleration.x, msg->acceleration.y, msg->acceleration.z;

  msf_core::Vector3 angvel;
  angvel << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;

  process_imu(linacc, angvel, msg->header.stamp, msg->header.seq);

}
template<typename EKFState_T>
void MSF_Core<EKFState_T>::imuCallback(const sensor_msgs::ImuConstPtr & msg)
{
  //  static int lastseq = -1;
  //  if((int)msg->header.seq != lastseq + 1 && lastseq != -1){
  //      ROS_WARN_STREAM("msf_core: imu message drop curr seq:"<<msg->header.seq<<" expected: "<<lastseq + 1);
  //  }
  //  lastseq = msg->header.seq;


//    if(msg->header.seq % 10 != 0){
//      ROS_WARN_STREAM_THROTTLE(60, "IMU throttling is on now!!!");
//      return;
//    }

  msf_core::Vector3 linacc;
  linacc << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;

  //  ROS_WARN_STREAM("core linacc: "<<linacc.transpose());

  msf_core::Vector3 angvel;
  angvel << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;

  //  ROS_WARN_STREAM("core  angvel: "<<  angvel.transpose());

  process_imu(linacc, angvel, msg->header.stamp, msg->header.seq);
}

template<typename EKFState_T>
void MSF_Core<EKFState_T>::process_imu(const msf_core::Vector3& linear_acceleration, const msf_core::Vector3& angular_velocity,
                                       const ros::Time& msg_stamp, size_t msg_seq)
                                       {
  //ROS_INFO_STREAM("MSF_Core: process_imu :"<<msf_core::timehuman(msg_stamp.toSec()));

  if(!initialized_){
    ROS_INFO_STREAM("IMU rejected not initialized");
    return;
  }

  sm::timing::Timer timer_PropgetClosestState("PropgetClosestState");
  boost::shared_ptr<EKFState_T> lastState = StateBuffer_.getClosestBefore(msg_stamp.toSec());
  timer_PropgetClosestState.stop();

  sm::timing::Timer timer_PropPrepare("PropPrepare");
  if(lastState->time == -1){
    ROS_WARN_STREAM_THROTTLE(2, "ImuCallback: closest state is invalid\n");
    return; // // early abort // //
  }

  boost::shared_ptr<EKFState_T> currentState(new EKFState_T);
  currentState->time = msg_stamp.toSec();

  //check if this IMU message is really after the last one (caused by restarting a bag file)
  if (currentState->time - lastState->time < -0.01 && predictionMade_){
    initialized_ = false;
    predictionMade_ = false;
    ROS_ERROR_STREAM("latest IMU message was out of order by a too large amount, resetting EKF: "
        "last-state-time: "<<msf_core::timehuman(lastState->time)<<" "<<
        "current-imu-time: "<<msf_core::timehuman(currentState->time));
    return;
  }

  static int seq = 0;
  // get inputs
  currentState->a_m = linear_acceleration;
  currentState->w_m = angular_velocity;

  // remove acc spikes (TODO: find a cleaner way to do this)
  static Eigen::Matrix<double, 3, 1> last_am = Eigen::Matrix<double, 3, 1>(0, 0, 0);
  if (currentState->a_m.norm() > 50)
    currentState->a_m = last_am;
  else{
    //try to get the state before the current time
    if(lastState->time == -1){
      ROS_WARN_STREAM("Accelerometer readings had a spike, but no prior state was in the buffer to take cleaner measurements from");
      return;
    }
    last_am = lastState->a_m;
  }
  if (!predictionMade_)
  {
    if(lastState->time == -1){
      ROS_WARN_STREAM("Wanted to compare prediction time offset to last state, but no prior state was in the buffer to take cleaner measurements from");
      return;
    }
    if (fabs(currentState->time - lastState->time) > 0.1)
    {
      ROS_WARN_STREAM_THROTTLE(2, "large time-gap re-initializing to last state\n");
      typename stateBufferT::Ptr_T tmp = StateBuffer_.updateTime(lastState->time, currentState->time);
      time_P_propagated = currentState->time;
      return; // // early abort // // (if timegap too big)
    }
  }

  if(lastState->time == -1){
    ROS_WARN_STREAM("Wanted to propagate state, but no valid prior state could be found in the buffer");
    return;
  }
  timer_PropPrepare.stop();

  sm::timing::Timer timer_PropState("PropState");
  //propagate state and covariance
  propagateState(lastState, currentState);
  timer_PropState.stop();
  sm::timing::Timer timer_PropCov("PropCov");
  propagatePOneStep();

  timer_PropCov.stop();


  // TODO Matlab back
//  if(StateBuffer_.size() > 3) //making sure we have sufficient states to apply measurements to
  predictionMade_ = true;
  ////////////////////////////
  //  ROS_WARN_STREAM("pred made: "<<predictionMade_<<" imu stamp: "<<timehuman(currentState->time));

  msgPose_.header.stamp = msg_stamp;
  msgPose_.header.seq = msg_seq;
  msgPose_.header.frame_id = "/world";

  currentState->toPoseMsg(msgPose_);
  pubPose_.publish(msgPose_);

  msgPoseCtrl_.header = msgPose_.header;
  currentState->toExtStateMsg(msgPoseCtrl_);
  pubPoseCrtl_.publish(msgPoseCtrl_);

  sm::timing::Timer timer_PropInsertState("PropInsertState");
  StateBuffer_.insert(currentState);
  timer_PropInsertState.stop();

  if(predictionMade_){
    handlePendingMeasurements(); //check if we can apply some pending measurement
  }
  seq++;

  //ROS_INFO_STREAM("msf_core process_imu done, pred. state:"<<std::endl<<currentState->print());



                                       }


template<typename EKFState_T>
void MSF_Core<EKFState_T>::stateCallback(const sensor_fusion_comm::ExtEkfConstPtr & msg)
{
  if(!initialized_)
    return;

  //get the closest state and check validity
  boost::shared_ptr<EKFState_T> lastState = StateBuffer_.getClosestBefore(msg->header.stamp.toSec());
  if(lastState->time == -1){
    ROS_WARN_STREAM_THROTTLE(2, "StateCallback: closest state is invalid\n");
    return; // // early abort // //
  }

  //create a new state
  boost::shared_ptr<EKFState_T> currentState(new EKFState_T);
  currentState->time = msg->header.stamp.toSec();

  // get inputs
  currentState->a_m << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
  currentState->w_m << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;

  // remove acc spikes (TODO: find a cleaner way to do this)
  static Eigen::Matrix<double, 3, 1> last_am = Eigen::Matrix<double, 3, 1>(0, 0, 0);
  if (currentState->a_m.norm() > 50)
    currentState->a_m = last_am;
  else
    last_am = currentState->a_m;

  if (!predictionMade_)
  {
    if (fabs(currentState->time - lastState->time) > 5)
    {
      typename stateBufferT::Ptr_T tmp = StateBuffer_.updateTime(lastState->time, currentState->time);
      ROS_WARN_STREAM_THROTTLE(2, "large time-gap re-initializing to last state: "<<msf_core::timehuman(tmp->time));
      return; // // early abort // // (if timegap too big)
    }
  }

  int32_t flag = msg->flag;
  if (data_playback_)
    flag = sensor_fusion_comm::ExtEkf::ignore_state;

  bool isnumeric = true;
  if (flag == sensor_fusion_comm::ExtEkf::current_state)
    isnumeric = checkForNumeric(Eigen::Map<const Eigen::Matrix<float, 10, 1> >(msg->state.data()), "before prediction p,v,q");

  isnumeric = checkForNumeric(currentState-> template get<StateDefinition_T::p>(), "before prediction p");

  if (flag == sensor_fusion_comm::ExtEkf::current_state && isnumeric) // state propagation is made externally, so we read the actual state
  {
    currentState-> template get<StateDefinition_T::p>() = Eigen::Matrix<double, 3, 1>(msg->state[0], msg->state[1], msg->state[2]);
    currentState-> template get<StateDefinition_T::v>() = Eigen::Matrix<double, 3, 1>(msg->state[3], msg->state[4], msg->state[5]);
    currentState-> template get<StateDefinition_T::q>() = Eigen::Quaternion<double>(msg->state[6], msg->state[7], msg->state[8], msg->state[9]);
    currentState-> template get<StateDefinition_T::q>().normalize();


    //	ROS_INFO_STREAM("TEST p_!!!");
    //	ROS_INFO_STREAM(currentState-> template get<StateDefinition_T::p>());
    ////	ROS_INFO_STREAM("p_ "<<(state.get<StateDefinition_T::p>()));



    // zero props: copy non propagation states from last state
    boost::fusion::for_each(
        currentState->statevars,
        msf_tmp::copyNonPropagationStates<EKFState_T>(*lastState)
    );

    hl_state_buf_ = *msg;
  }
  else if (flag == sensor_fusion_comm::ExtEkf::ignore_state || !isnumeric){ // otherwise let's do the state prop. here
    propagateState(lastState, currentState);
  }

  propagatePOneStep();

  isnumeric = checkForNumeric(currentState-> template get<StateDefinition_T::p>(), "prediction p");
  isnumeric = checkForNumeric(currentState->P, "prediction done P");

  if(!predictionMade_){ //clean reset of state and measurement buffer, before we start propagation

    currentState->P = StateBuffer_.getLast()->P;//make sure we keep the covariance for the first state
    time_P_propagated = currentState->time;

    StateBuffer_.clear();
    MeasurementBuffer_.clear();
    while(!queueFutureMeasurements_.empty()){
      queueFutureMeasurements_.pop();
    }
  }
  predictionMade_ = true;

  StateBuffer_.insert(currentState);
  handlePendingMeasurements(); //check if we can apply some pending measurement
}

template<typename EKFState_T>
void MSF_Core<EKFState_T>::handlePendingMeasurements()
{
  if(queueFutureMeasurements_.empty())
    return;
  boost::shared_ptr<MSF_MeasurementBase<EKFState_T> > meas = queueFutureMeasurements_.front();
  queueFutureMeasurements_.pop();
  addMeasurement(meas);
}


template<typename EKFState_T>
void MSF_Core<EKFState_T>::propagateState(boost::shared_ptr<EKFState_T>& state_old, boost::shared_ptr<EKFState_T>& state_new)
{

  //ROS_INFO_STREAM("MSF_Core: propagateState from:"<<msf_core::timehuman(state_old->time)<<" to "<<msf_core::timehuman(state_new->time));

  bool debug = false;


  //	  ROS_INFO_STREAM("STATES before propagation step:");
  //	  std::cout<<state_old->print()<<std::endl;

  //	  ROS_INFO_STREAM("P before propagation: "<<state_new->P);

  double dt = state_new->time - state_old->time;

  //reset new state to zero
  boost::fusion::for_each(
      state_new->statevars,
      msf_tmp::resetState()
  );


  // zero props: copy constant for non propagated states
  boost::fusion::for_each(
      state_new->statevars,
      msf_tmp::copyNonPropagationStates<EKFState_T>(*state_old)
  );

//  ROS_INFO_STREAM_COND(debug, "w_m: "<<state_new->w_m);
//  ROS_INFO_STREAM_COND(debug, "a_m: "<<state_new->a_m);
//  ROS_INFO_STREAM_COND(debug, "C_est_transposed = " << state_old-> template get<StateDefinition_T::q>().conjugate().toRotationMatrix());


  //TODO Matlab back (done)
//  Eigen::Matrix<double,4,1> stream_qbeforepred =  state_old-> template get<StateDefinition_T::q>().coeffs();
//  ROS_INFO_STREAM("C_est_transposed = " << state_old-> template get<StateDefinition_T::q>().conjugate().toRotationMatrix());
//

  /////////////////////

  //ROS_INFO_STREAM("C_est_transposed = "<<state_old-> template get<StateDefinition_T::q>().toRotationMatrix());

  //  // DB test
  //  state_new-> template get<StateDefinition_T::b_a>()(0) = 0;
  //  state_new-> template get<StateDefinition_T::b_a>()(1) = 0;

  Eigen::Matrix<double, 3, 1> dv;
  const Vector3 ew = state_new->w_m - state_new-> template get<StateDefinition_T::b_w>();
  const Vector3 ewold = state_old->w_m - state_old-> template get<StateDefinition_T::b_w>();
  const Vector3 ea = state_new->a_m - state_new-> template get<StateDefinition_T::b_a>();
  const Vector3 eaold = state_old->a_m - state_old-> template get<StateDefinition_T::b_a>();


  //TODO Matlab back
//  const Matrix4 Omega = omegaMatJPL(ew);

  const Matrix4 Omega = omegaMatJPL(state_new->w_m - state_new-> template get<StateDefinition_T::b_w>());

  Vector3 wm_bwest = state_new->w_m - state_new-> template get<StateDefinition_T::b_w>();
  Vector3 wm = state_new->w_m;
  Vector3 bwest= state_new-> template get<StateDefinition_T::b_w>();

//  ROS_INFO_STREAM( "wm_bwest: "<< wm_bwest);
//  ROS_INFO_STREAM( "wm: "<< wm);
//  ROS_INFO_STREAM("am: "<< state_new->a_m);
//  ROS_INFO_STREAM( "bwest: "<< bwest);


//  ROS_INFO_STREAM( "Omega: "<<Omega);
//  ROS_INFO_STREAM("dt: "<<dt);

  /////////////////////////////////////




  const Matrix4 OmegaOld = omegaMatJPL(ewold);
  Matrix4 OmegaMean = omegaMatJPL((ew + ewold) / 2);



  //  // zero order quaternion integration
  //  //	cur_state.q_ = (Eigen::Matrix<double,4,4>::Identity() + 0.5*Omega*dt)*StateBuffer_[(unsigned char)(idx_state_-1)].q_.coeffs();
  //
  //  ROS_INFO_STREAM("q before quaternion integration: "<<state_old-> template get<StateDefinition_T::q>().coeffs());
  //
  //  ROS_INFO_STREAM("Omega: "<<Omega);




  //TODO Matlab back
  state_new-> template get<StateDefinition_T::q>().coeffs() = (Eigen::Matrix<double,4,4>::Identity() + 0.5*Omega*dt)*state_old-> template get<StateDefinition_T::q>().coeffs();
  state_new-> template get<StateDefinition_T::q>().normalize();



  Eigen::Matrix<double,4,1> stream_qpred =  (Eigen::Matrix<double,4,4>::Identity() + 0.5*Omega*dt)*state_old-> template get<StateDefinition_T::q>().coeffs();

  //
  //  ROS_INFO_STREAM("q after quaternion integration: "<<state_new-> template get<StateDefinition_T::q>().coeffs());

  /////////////////////////



  //TODO Matlab back

  //  // first order quaternion integration, this is kind of costly and may not add a lot to the quality of propagation...
  //  int div = 1;
  //  Matrix4 MatExp;
  //  MatExp.setIdentity();
  //  OmegaMean *= 0.5 * dt;
  //  for (int i = 1; i < 5; i++) //can be made fourth order or less
  //  {
  //    div *= i;
  //    MatExp = MatExp + OmegaMean / div;
  //    OmegaMean *= OmegaMean;
  //  }
  //
  //  // first oder quat integration matrix
  //  const Matrix4 quat_int = MatExp + 1.0 / 48.0 * (Omega * OmegaOld - OmegaOld * Omega) * dt * dt;
  //
  //  // first oder quaternion integration
  //  state_new-> template get<StateDefinition_T::q>().coeffs() = quat_int * state_old-> template get<StateDefinition_T::q>().coeffs();
  //  state_new-> template get<StateDefinition_T::q>().normalize();
  //
  ///////////////////////////////////////


  //TODO Matlab back
  //  dv = (state_new-> template get<StateDefinition_T::q>().toRotationMatrix() * ea + state_old-> template get<StateDefinition_T::q>().toRotationMatrix() * eaold) / 2;

  dv = state_old-> template get<StateDefinition_T::q>().toRotationMatrix() * ea;
  ///////////////////////////////////////////

  ROS_INFO_STREAM_COND(debug, "ea: "<<ea);
  ROS_INFO_STREAM_COND(debug, "dv: "<<dv);


  //TODO Matlab back



  state_new-> template get<StateDefinition_T::v>() = state_old-> template get<StateDefinition_T::v>() + (dv - g_) * dt;
//  state_new-> template get<StateDefinition_T::p>() = state_old-> template get<StateDefinition_T::p>() + ((state_new-> template get<StateDefinition_T::v>() + state_old-> template get<StateDefinition_T::v>()) / 2 * dt);

//  r_est_new = r_est + dt*v_est + dt^2/2*(C_est'*(a_m - b_a_est) + grav);
//  v_est_new = v_est + dt*(C_est'*(a_m - b_a_est) + grav);


  Vector3 p_old = state_old-> template get<StateDefinition_T::p>();
  Vector3 v_old = state_old-> template get<StateDefinition_T::v>();


  state_new-> template get<StateDefinition_T::p>() = state_old-> template get<StateDefinition_T::p>() + state_old-> template get<StateDefinition_T::v>() * dt + (dv - g_) * dt * dt /2;


  Eigen::Matrix<double,3,1> stream_ppred = state_old-> template get<StateDefinition_T::p>() + state_old-> template get<StateDefinition_T::v>() * dt + (dv - g_) * dt * dt /2;
  Eigen::Matrix<double,3,1> stream_vpred = state_old-> template get<StateDefinition_T::v>() + (dv - g_) * dt;

//  ROS_INFO_STREAM("dv: " << dv);
//  ROS_INFO_STREAM("p_old: " << p_old);
//  ROS_INFO_STREAM("v_old: " << v_old);
//  ROS_INFO_STREAM("p_pred: " << stream_ppred);
//  ROS_INFO_STREAM("v_pred: " << stream_vpred);
//  ROS_INFO_STREAM("q_pred: " << stream_qpred);



  //////////////////////////////////////////

  tf::Transform transform;
  Eigen::Matrix<double, 3, 1>& pos = state_new-> template get<StateDefinition_T::p>();
  Eigen::Quaterniond& ori = state_new-> template get<StateDefinition_T::q>();
  transform.setOrigin( tf::Vector3(pos[0], pos[1], pos[2]) );
  transform.setRotation( tf::Quaternion(ori.x(), ori.y(), ori.z(), ori.w()) );
  tf_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now() /*ros::Time(latestState->time_)*/, "world", "state"));

#ifdef DEBUGPUBLISH
  static int seq = 0;
  msgState_.header.seq = seq++;
  msgState_.header.stamp = ros::Time(state_new->time);
  state_new->toFullStateMsg(msgState_);
  pubState_.publish(msgState_);
#endif

  //  ROS_INFO_STREAM("STATES after propagation step:");
  //  		std::cout<<state_new->print()<<std::endl;

  //  		ROS_INFO_STREAM("P after propagation: "<<state_new->P);


}

template<typename EKFState_T>
void MSF_Core<EKFState_T>::propagatePOneStep(){
  //also propagate the covariance one step further, to distribute the processing load over time
  typename stateBufferT::iterator_T stateIteratorPLastPropagated = StateBuffer_.getIteratorAtValue(time_P_propagated, false);
  typename stateBufferT::iterator_T stateIteratorPLastPropagatedNext = stateIteratorPLastPropagated;

  ++stateIteratorPLastPropagatedNext;
  if(stateIteratorPLastPropagatedNext != StateBuffer_.getIteratorEnd()){ //might happen if there is a measurement in the future


    // TODO Matlab back (done)
//    ROS_INFO_STREAM("propPOneStep!!!");
    //////////////////////////////////////

    predictProcessCovariance(stateIteratorPLastPropagated->second, stateIteratorPLastPropagatedNext->second);

    if(!checkForNumeric(stateIteratorPLastPropagatedNext->second-> template get<StateDefinition_T::p>(), "prediction p")){
      ROS_WARN_STREAM("prop state from:\t"<<stateIteratorPLastPropagated->second->toEigenVector());
      ROS_WARN_STREAM("prop state to:\t"<<stateIteratorPLastPropagatedNext->second->toEigenVector());
      ROS_ERROR_STREAM("Resetting EKF");
      predictionMade_ = initialized_ = false;
    }
  }
}

template<typename EKFState_T>
void MSF_Core<EKFState_T>::getAccumF_SC(const boost::shared_ptr<EKFState_T>& state_old, const boost::shared_ptr<EKFState_T>& state_new, Eigen::Matrix<double,
                                        MSF_Core<EKFState_T>::nErrorStatesAtCompileTime, MSF_Core<EKFState_T>::nErrorStatesAtCompileTime>& F){
  typename stateBufferT::iterator_T it = StateBuffer_.getIteratorAtValue(state_old);
  typename stateBufferT::iterator_T itend = StateBuffer_.getIteratorAtValue(state_new);
  typedef Eigen::Matrix<double, MSF_Core<EKFState_T>::nErrorStatesAtCompileTime, MSF_Core<EKFState_T>::nErrorStatesAtCompileTime> F_type;
  F = F_type::Identity();
  for(; it != itend; ++it){
    F = F * it->second->Fd;
  }
}

template<typename EKFState_T>
void MSF_Core<EKFState_T>::predictProcessCovariance(boost::shared_ptr<EKFState_T>& state_old, boost::shared_ptr<EKFState_T>& state_new)
{

  double dt = state_new->time - state_old->time;

  if(dt <= 0){
    ROS_WARN_STREAM_THROTTLE(1,"Requested cov prop between two states that where "<<dt<<" seconds apart. Rejecting");
    return;
  }

  //Eigen::Matrix<double, nErrorStatesAtCompileTime, nErrorStatesAtCompileTime> Fd(Eigen::Matrix<double, nErrorStatesAtCompileTime, nErrorStatesAtCompileTime>::Identity()); ///< discrete state propagation matrix

  // noises
  const Vector3 nav = Vector3::Constant(usercalc_.getParam_noise_acc() /* / sqrt(dt) */);
  const Vector3 nbav = Vector3::Constant(usercalc_.getParam_noise_accbias() /* * sqrt(dt) */);

  const Vector3 nwv = Vector3::Constant(usercalc_.getParam_noise_gyr() /* / sqrt(dt) */);
  const Vector3 nbwv = Vector3::Constant(usercalc_.getParam_noise_gyrbias() /* * sqrt(dt) */);

  // bias corrected IMU readings
  const Vector3 ew = state_new->w_m - state_new-> template get<StateDefinition_T::b_w>();
  const Vector3 ea = state_new->a_m - state_new-> template get<StateDefinition_T::b_a>();

  const Matrix3 a_sk = skew(ea);
  const Matrix3 w_sk = skew(ew);
  const Matrix3 eye3 = Eigen::Matrix<double, 3, 3>::Identity();

  const Matrix3 C_eq = state_old-> template get<StateDefinition_T::q>().toRotationMatrix();

  const double dt_p2_2 = dt * dt * 0.5; // dt^2 / 2
  const double dt_p3_6 = dt_p2_2 * dt / 3.0; // dt^3 / 6
  const double dt_p4_24 = dt_p3_6 * dt * 0.25; // dt^4 / 24
  const double dt_p5_120 = dt_p4_24 * dt * 0.2; // dt^5 / 120

  const Matrix3 Ca3 = C_eq * a_sk;
  const Matrix3 A = Ca3 * (-dt_p2_2 * eye3 + dt_p3_6 * w_sk - dt_p4_24 * w_sk * w_sk);
  const Matrix3 B = Ca3 * (dt_p3_6 * eye3 - dt_p4_24 * w_sk + dt_p5_120 * w_sk * w_sk);
  const Matrix3 D = -A;
  const Matrix3 E = eye3 - dt * w_sk + dt_p2_2 * w_sk * w_sk;
  const Matrix3 F = -dt * eye3 + dt_p2_2 * w_sk - dt_p3_6 * (w_sk * w_sk);
  const Matrix3 C = Ca3 * F;

  // discrete error state propagation Matrix Fd according to:
  // Stephan Weiss and Roland Siegwart.
  // Real-Time Metric State Estimation for Modular Vision-Inertial Systems.
  // IEEE International Conference on Robotics and Automation. Shanghai, China, 2011
  typename EKFState_T::F_type& Fd = state_old->Fd;


  // TODO Matlab back
  Fd. template block<3, 3> (0, 3) = dt * eye3;
  Fd. template block<3, 3> (0, 6) = A;
  Fd. template block<3, 3> (0, 9) = B;
  Fd. template block<3, 3> (0, 12) = -C_eq * dt_p2_2;

  Fd. template block<3, 3> (3, 6) = C;
  Fd. template block<3, 3> (3, 9) = D;
  Fd. template block<3, 3> (3, 12) = -C_eq * dt;

  Fd. template block<3, 3> (6, 6) = E;
  Fd. template block<3, 3> (6, 9) = F;



//  const Eigen::MatrixXd eye = Eigen::Matrix<double, nErrorStatesAtCompileTime, nErrorStatesAtCompileTime>::Identity();
//  Fd. template setZero();
//
//  Fd. template block<3, 3> (0, 3) = eye3;
//
//  Fd. template block<3, 3> (3, 6) = - C_eq * skew(ea);
//  Fd. template block<3, 3> (3, 12) = - C_eq;
//
//  Fd. template block<3, 3> (6, 6) = - skew(ew);
//  Fd. template block<3, 3> (6, 9) = - eye3;
//
//
//  Fd = (eye + Fd * dt);// + Fd * Fd * (dt*dt/2)); // + Fd * Fd * Fd * (dt*dt*dt/6));


//  ROS_INFO_STREAM("w_m for F,Q,P: " << state_new->w_m);
//  ROS_INFO_STREAM("a_m for F,Q,P: " << state_new->a_m);

//  ROS_INFO_STREAM("dt: " << dt);
//  ROS_INFO_STREAM("C_eq: " << C_eq);
//  ROS_INFO_STREAM("Fd: " << Fd);
  //////////////////////////////////////////

  typename EKFState_T::Q_type& Qd = state_old->Qd;

  //Eigen::Matrix<double, nErrorStatesAtCompileTime, nErrorStatesAtCompileTime> Qd(Eigen::Matrix<double, nErrorStatesAtCompileTime, nErrorStatesAtCompileTime>::Zero()); ///< discrete propagation noise matrix
  calc_QCore(dt, state_old-> template get<StateDefinition_T::q>(), ew, ea, nav, nbav, nwv, nbwv, Qd);

  //call user Q calc to fill in the blocks of auxiliary states
  //TODO optim: make state Q-blocks map respective parts of Q using Eigen Map, avoids copy
  usercalc_.calculateQAuxiliaryStates(*state_old, dt);

  //now copy the userdefined blocks to Qd
  boost::fusion::for_each(
      state_old->statevars,
      msf_tmp::copyQBlocksFromAuxiliaryStatesToQ<StateSequence_T>(Qd)
  );

  //TODO optim: multiplication of F blockwise, using the fact that aux states have no entries outside their block



  // TODO Matlab back
//  state_new->P = Fd * state_old->P * Fd.transpose() + Qd;
  state_new->P = Fd * state_old->P * Fd.transpose() + Qd;

//  ROS_INFO_STREAM("P_PREV: "<<state_old->P);
//  ROS_INFO_STREAM("P_pred: "<<state_new->P);


  //////////////////////


  //set time for best cov prop to now
  time_P_propagated = state_new->time;

}

template<typename EKFState_T>
void MSF_Core<EKFState_T>::init(boost::shared_ptr<MSF_MeasurementBase<EKFState_T> > measurement){

  initialized_ = false;
  predictionMade_ = false;

  std::this_thread::sleep_for(std::chrono::milliseconds(100)); //hackish thread sync

  MeasurementBuffer_.clear();
  StateBuffer_.clear();
  fuzzyTracker_.reset();

  while(!queueFutureMeasurements_.empty())
    queueFutureMeasurements_.pop();

  //push one state to the buffer to apply the init on
  boost::shared_ptr<EKFState_T> state(new EKFState_T);
  state->time = 0; //will be set by the measurement

  //reset new state to zero
  boost::fusion::for_each(
      state->statevars,
      msf_tmp::resetState()
  );

  //set intialial covariance for core states
  setPCore(state->P);

  //apply init measurement, where the user can provide additional values for P
  measurement->apply(state, *this);

  std::this_thread::sleep_for(std::chrono::milliseconds(100)); //wait for the external propagation to get the init message

  assert(state->time != 0);

  StateBuffer_.insert(state);
  time_P_propagated = state->time; //will be set upon first IMU message

  //echo params
  ROS_INFO_STREAM("Core parameters:"<<std::endl<<
                  "\tfixed_bias:\t"<<usercalc_.getParam_fixed_bias()<<std::endl<<
                  "\tfuzzythres:\t"<<usercalc_.getParam_fuzzythres()<<std::endl<<
                  "\tnoise_acc:\t"<<usercalc_.getParam_noise_acc()<<std::endl<<
                  "\tnoise_accbias:\t"<<usercalc_.getParam_noise_accbias()<<std::endl<<
                  "\tnoise_gyr:\t"<<usercalc_.getParam_noise_gyr()<<std::endl<<
                  "\tnoise_gyrbias:\t"<<usercalc_.getParam_noise_gyrbias()<<std::endl);


  ROS_INFO_STREAM("core init with state:"<<std::endl<<state->print());
  initialized_ = true;

  sm::timing::Timing::print(std::cout);
}

template<typename EKFState_T>
void MSF_Core<EKFState_T>::addMeasurement(boost::shared_ptr<MSF_MeasurementBase<EKFState_T> > measurement){

  ROS_INFO_STREAM("Core: add measurement "<<msf_core::timehuman(measurement->time));

  if(!initialized_ || !predictionMade_){
    ROS_WARN_STREAM("Rejected, no prediction"<<predictionMade_<<" or not initialized "<<initialized_);
    return;
  }

  if(measurement->time > StateBuffer_.getLast()->time){
    ROS_WARN_STREAM("You tried to give me a measurement in the future. Are you sure your clocks are synced and delays compensated correctly? I will store that and apply it next time... [measurement: "<<timehuman(measurement->time)<<" (s) latest state: "<<timehuman(StateBuffer_.getLast()->time)<<" (s)]");
    queueFutureMeasurements_.push(measurement);
    return;
  }
  if(measurement->time < StateBuffer_.getFirst()->time){
    ROS_WARN_STREAM("You tried to give me a measurement which is too far in the past. Are you sure your clocks are synced and delays compensated correctly? [measurement: "<<timehuman(measurement->time)<<" (s) first state in buffer: "<<timehuman(StateBuffer_.getFirst()->time)<<" (s)]");
    return; //reject measurements too far in the past
  }

  typename measurementBufferT::iterator_T it_meas = MeasurementBuffer_.insert(measurement);
  typename measurementBufferT::iterator_T it_meas_end = MeasurementBuffer_.getIteratorEnd();
  typename stateBufferT::iterator_T it_curr = StateBuffer_.getIteratorEnd(); //no propagation if no update is applied

  bool appliedOne = false;

  isfuzzyState_ = false;

  for( ; it_meas != it_meas_end; ++it_meas){

    if(it_meas->second->time <= 0) //valid?
      continue;
    sm::timing::Timer timer_meas_get_state("Get state for measurement");
    boost::shared_ptr<EKFState_T> state = getClosestState(it_meas->second->time); //propagates covariance to state
    timer_meas_get_state.stop();
    if(state->time <= 0){
      ROS_ERROR_STREAM_THROTTLE(1,"getClosestState returned an invalid state");
      continue;
    }

    sm::timing::Timer timer_meas_apply("Apply measurement");
    it_meas->second->apply(state, *this); //calls back core::applyCorrection(), which sets time_P_propagated to meas time
    timer_meas_apply.stop();
    //make sure to propagate to next measurement or up to now if no more measurements
    it_curr = StateBuffer_.getIteratorAtValue(state); //propagate from current state

    typename stateBufferT::iterator_T it_end;
    typename measurementBufferT::iterator_T it_nextmeas = it_meas;
    ++it_nextmeas; //the next measurement in the list

    if(it_nextmeas == it_meas_end){ //that was the last measurement, so propagate state to now
      it_end = StateBuffer_.getIteratorEnd();
    }else{
      it_end = StateBuffer_.getIteratorClosestAfter(it_nextmeas->second->time);
      if(it_end != StateBuffer_.getIteratorEnd()){
        ++it_end; //propagate to state closest after the measurement so we can interpolate
      }
    }


    typename stateBufferT::iterator_T it_next = it_curr;
    ++it_next;

    sm::timing::Timer timer_prop_state_after_meas("Repropagate state to now");
    for( ; it_curr != it_end && it_next != it_end && it_curr->second->time != -1 && it_next->second->time != -1; ++it_curr, ++it_next){ //propagate to selected state
      if(it_curr->second == it_next->second){
        ROS_ERROR_STREAM("propagation : it_curr points to same state as it_next. This must not happen.");
        continue;
      }
      if(!initialized_ || !predictionMade_) //break loop if EKF reset in the meantime
        return;
      propagateState(it_curr->second, it_next->second);
    }
    timer_prop_state_after_meas.stop();
    appliedOne = true;
  }

  if(!appliedOne){
    ROS_WARN_STREAM("no measurement was applied, this should not happen");
    return;
  }

  //now publish the best current estimate
  static int seq_m = 0;

  // publish correction for external propagation
  boost::shared_ptr<EKFState_T>& latestState = StateBuffer_.getLast();
  msgCorrect_.header.stamp = ros::Time(latestState->time);
  msgCorrect_.header.seq = seq_m;
  msgCorrect_.angular_velocity.x = 0;
  msgCorrect_.angular_velocity.y = 0;
  msgCorrect_.angular_velocity.z = 0;
  msgCorrect_.linear_acceleration.x = 0;
  msgCorrect_.linear_acceleration.y = 0;
  msgCorrect_.linear_acceleration.z = 0;

  // prevent junk being sent to the external state propagation when data playback is (accidentally) on
  if(data_playback_){
    for(int i=0; i<HLI_EKF_STATE_SIZE; ++i){
      msgCorrect_.state[i] = 0;
    }
    msgCorrect_.state[6] = 1;
    msgCorrect_.flag = sensor_fusion_comm::ExtEkf::initialization;


    if(pubCorrect_.getNumSubscribers() > 0){
      ROS_ERROR_STREAM_THROTTLE(1, "You have connected the external propagation topic but at the same time data_playback is on.");
    }

  }
  else{
    msgCorrect_.state[0] = latestState-> template get<StateDefinition_T::p>()[0] - hl_state_buf_.state[0];
    msgCorrect_.state[1] = latestState-> template get<StateDefinition_T::p>()[1] - hl_state_buf_.state[1];
    msgCorrect_.state[2] = latestState-> template get<StateDefinition_T::p>()[2] - hl_state_buf_.state[2];
    msgCorrect_.state[3] = latestState-> template get<StateDefinition_T::v>()[0] - hl_state_buf_.state[3];
    msgCorrect_.state[4] = latestState-> template get<StateDefinition_T::v>()[1] - hl_state_buf_.state[4];
    msgCorrect_.state[5] = latestState-> template get<StateDefinition_T::v>()[2] - hl_state_buf_.state[5];

    Eigen::Quaterniond hl_q(hl_state_buf_.state[6], hl_state_buf_.state[7], hl_state_buf_.state[8], hl_state_buf_.state[9]);
    Eigen::Quaterniond qbuff_q = hl_q.inverse() * latestState-> template get<StateDefinition_T::q>();
    msgCorrect_.state[6] = qbuff_q.w();
    msgCorrect_.state[7] = qbuff_q.x();
    msgCorrect_.state[8] = qbuff_q.y();
    msgCorrect_.state[9] = qbuff_q.z();

    msgCorrect_.state[10] = latestState-> template get<StateDefinition_T::b_w>()[0] - hl_state_buf_.state[10];
    msgCorrect_.state[11] = latestState-> template get<StateDefinition_T::b_w>()[1] - hl_state_buf_.state[11];
    msgCorrect_.state[12] = latestState-> template get<StateDefinition_T::b_w>()[2] - hl_state_buf_.state[12];
    msgCorrect_.state[13] = latestState-> template get<StateDefinition_T::b_a>()[0] - hl_state_buf_.state[13];
    msgCorrect_.state[14] = latestState-> template get<StateDefinition_T::b_a>()[1] - hl_state_buf_.state[14];
    msgCorrect_.state[15] = latestState-> template get<StateDefinition_T::b_a>()[2] - hl_state_buf_.state[15];

    msgCorrect_.flag = sensor_fusion_comm::ExtEkf::state_correction;
  }

  if(latestState->checkStateForNumeric()){ //if not NaN
    pubCorrect_.publish(msgCorrect_);
  }else{
    ROS_WARN_STREAM_THROTTLE(1, "Not sending updates to external EKF, because state NaN/inf");
  }

  // publish state
  msgState_.header = msgCorrect_.header;
  latestState->toFullStateMsg(msgState_);
  pubState_.publish(msgState_);

  if(pubPoseAfterUpdate_.getNumSubscribers()){
    //publish pose after correction with covariance
    propPToState(latestState); //get the covar

    msgPose_.header.stamp = ros::Time(latestState->time);
    msgPose_.header.seq = seq_m;
    msgPose_.header.frame_id = "/world";

    latestState->toPoseMsg(msgPose_);
    pubPoseAfterUpdate_.publish(msgPose_);
  }
  seq_m++;
}

template<typename EKFState_T>
boost::shared_ptr<msf_core::MSF_MeasurementBase<EKFState_T> > MSF_Core<EKFState_T>::getPreviousMeasurement(double time, int sensorID) {
  typename measurementBufferT::iterator_T it = MeasurementBuffer_.getIteratorAtValue(time);
  if(it->second->time != time){
    ROS_WARN_STREAM("getPreviousMeasurement: Error invalid iterator at value");
    return MeasurementBuffer_.getInvalid();
  }
  --it;
  typename measurementBufferT::iterator_T itbeforebegin = MeasurementBuffer_.getIteratorBeforeBegin();
  while(it != itbeforebegin && (it->second->time != time && it->second->sensorID_ != sensorID)){
    --it;
  }
  if(it == itbeforebegin){
    ROS_WARN_STREAM("getPreviousMeasurement: Error hit before begin");
    return MeasurementBuffer_.getInvalid();
  }
  return it->second;
}

template<typename EKFState_T>
boost::shared_ptr<EKFState_T> MSF_Core<EKFState_T>::getStateAtTime(double tstamp){
  return StateBuffer_.getValueAt(tstamp);
}

template<typename EKFState_T>
boost::shared_ptr<EKFState_T> MSF_Core<EKFState_T>::getClosestState(double tstamp)
{

  //sweiss{
  // we subtracted one too much before.... :)
  //}

  double timenow = tstamp; //delay compensated by sensor handler

  typename stateBufferT::iterator_T it = StateBuffer_.getIteratorClosest(timenow);

  boost::shared_ptr<EKFState_T> closestState = it->second;

  if (closestState->time == -1 || fabs(closestState->time - timenow) > 0.1){ //check if the state really is close to the requested time. With the new buffer this might not be given.
    ROS_ERROR_STREAM("Requested closest state to "<<timehuman(timenow)<<" but there was no suitable state in the map");
    return StateBuffer_.getInvalid(); // // early abort // //  not enough predictions made yet to apply measurement (too far in past)
  }

  //do state interpolation if state is too far away from the measurement
  double tdiff = fabs(closestState->time - timenow); //timediff to closest state


  //TODO Matlab back
//  if(tdiff > 0.001){ // if time diff too large, insert new state and do state interpolation
  if(0){ // if time diff too large, insert new state and do state interpolation
    ///////////////////////


    boost::shared_ptr<EKFState_T> lastState = StateBuffer_.getClosestBefore(timenow);
    boost::shared_ptr<EKFState_T> nextState = StateBuffer_.getClosestAfter(timenow);

    bool statevalid = lastState->time != -1 && nextState->time != -1;
    bool statenotnan = lastState->checkStateForNumeric() && nextState->checkStateForNumeric();
    bool statesnotsame = lastState->time != nextState->time;

    if(statevalid && statenotnan && statesnotsame){ //if one of the states is invalid, we don't do interpolation, but just take closest

      //prepare a new state
      boost::shared_ptr<EKFState_T> currentState(new EKFState_T);
      currentState->time = timenow; //set state time to measurement time
      //linearly interpolate imu readings
      currentState->a_m = lastState->a_m + (nextState->a_m - lastState->a_m) / (nextState->time - lastState->time) * (timenow - lastState->time);
      currentState->w_m = lastState->w_m + (nextState->w_m - lastState->w_m) / (nextState->time - lastState->time) * (timenow - lastState->time);

      propagateState(lastState, currentState); //propagate with respective dt

      StateBuffer_.insert(currentState);

      if(time_P_propagated > lastState->time){ //make sure we propagate P correctly to the new state
        time_P_propagated = lastState->time;
      }

      closestState = currentState;
    }
  }

  propPToState(closestState); // catch up with covariance propagation if necessary

  if(!closestState->checkStateForNumeric()){
    ROS_ERROR_STREAM("State interpolation: interpolated state is invalid (nan)");
    return StateBuffer_.getInvalid(); // // early abort // //
  }

  static int janitorRun = 0;
  if(janitorRun++ > 100){
    CleanUpBuffers(); //remove very old states and measurements from the buffers
    janitorRun = 0;
  }
  return closestState;
}

template<typename EKFState_T>
void MSF_Core<EKFState_T>::propPToState(boost::shared_ptr<EKFState_T>& state)
{

  // TODO Matlab back (done)
//  ROS_INFO_STREAM("propPToState!!!");
//  ///////////////


  // propagate cov matrix until the current states time
  typename stateBufferT::iterator_T it = StateBuffer_.getIteratorAtValue(time_P_propagated, false);
  typename stateBufferT::iterator_T itMinus = it;
  ++it;
  //until we reached the current state or the end of the state list
  for( ; it != StateBuffer_.getIteratorEnd() && it->second->time <= state->time ; ++it, ++itMinus){
    ROS_INFO_STREAM("Propagating state from "<<itMinus->second->time<<" to "<<it->second->time);
    predictProcessCovariance(itMinus->second, it->second);
  }

  //////////////////////////////////////////////


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
    typedef typename msf_tmp::getEnumStateType<StateSequence_T, StateDefinition_T::b_a>::value b_a_type;
    typedef typename msf_tmp::getEnumStateType<StateSequence_T, StateDefinition_T::b_w>::value b_w_type;

    enum{
      indexOfState_b_a = msf_tmp::getStartIndex<StateSequence_T, b_a_type, msf_tmp::CorrectionStateLengthForType>::value,
      indexOfState_b_w = msf_tmp::getStartIndex<StateSequence_T, b_w_type, msf_tmp::CorrectionStateLengthForType>::value
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

  //TODO: allow multiple fuzzy tracking states at the same time
  isfuzzyState_ |= fuzzyTracker_.check(delaystate, buffstate, fuzzythres);

  //no publishing and propagation here, because this might not be the last update we have to apply

  checkForNumeric(correction, "update");

  time_P_propagated = delaystate->time; //set time latest propagated, we need to repropagate at least from here

  publishCovImage(delaystate); //publish cov image

  return 1;
}




}; // end namespace
