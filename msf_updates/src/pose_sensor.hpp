/*

Copyright (c) 2010, Stephan Weiss, ASL, ETH Zurich, Switzerland
You can contact the author at <stephan dot weiss at ieee dot org>

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

#include "pose_sensor.h"
#include <msf_core/eigen_utils.h>

#define N_MEAS 7 // measurement size
PoseSensorHandler::PoseSensorHandler(msf_core::MSF_SensorManager* meas) :
  SensorHandler(meas)
{
  ros::NodeHandle pnh("~");
  pnh.param("measurement_world_sensor", measurement_world_sensor_, true);
  pnh.param("use_fixed_covariance", use_fixed_covariance_, false);

  ROS_INFO_COND(measurement_world_sensor_, "interpreting measurement as sensor w.r.t. world");
  ROS_INFO_COND(!measurement_world_sensor_, "interpreting measurement as world w.r.t. sensor (e.g. ethzasl_ptam)");

  ROS_INFO_COND(use_fixed_covariance_, "using fixed covariance");
  ROS_INFO_COND(!use_fixed_covariance_, "using covariance from sensor");

  subscribe();
}

void PoseSensorHandler::subscribe()
{
  ros::NodeHandle nh("msf_core");
  subMeasurement_ = nh.subscribe("pose_measurement", 1, &PoseSensorHandler::measurementCallback, this);
  subPressure_ = nh.subscribe("pressure", 1, &PoseSensorHandler::pressureCallback, this);

  //TODO: we might need to implement that for initial testing
  //measurements->msf_core_.registerCallback(&PoseSensorHandler::noiseConfig, this);

  nh.param("meas_noise1", n_zp_, 0.01);	// default position noise is for ethzasl_ptam
  nh.param("meas_noise2", n_zq_, 0.02);	// default attitude noise is for ethzasl_ptam

  pressure_offset_=0;
}

void PoseSensorHandler::noiseConfig(msf_core::MSF_CoreConfig& config, uint32_t level)
{
  //	if(level & msf_core::MSF_Core_MISC)
  //	{
	  this->n_zp_ = config.meas_noise1;
	  this->n_zq_ = config.meas_noise2;
  //	}
  	if(level & msf_core::MSF_Core_RESET_PRESS)
  	{
  		pressure_offset_=measurements->press_height_;
	}
}

void PoseSensorHandler::pressureCallback(const asctec_hl_comm::mav_imuConstPtr & msg)
{
	static double heightbuff[10]={0,0,0,0,0,0,0,0,0,0};
	memcpy(heightbuff, heightbuff+1, sizeof(double)*9);
	heightbuff[9] = msg->height;
	measurements->press_height_=0;
	for(int k=0; k<10; ++k)
		measurements->press_height_+=heightbuff[k];
	measurements->press_height_ /=10;
	//ROS_WARN_STREAM("Got pressure measurement "<<-msg->height<< " => "<<measurements->press_height_);

}

void PoseSensorHandler::measurementCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr & msg)
{
  	ROS_INFO_STREAM("measurement received \n"
  					<< "type is: " << typeid(msg).name());

  // init variables
  msf_core::EKFState state_old_o;
  ros::Time time_old = msg->header.stamp;

  Eigen::Matrix<double, N_MEAS, msf_core::MSF_Core::nErrorStatesAtCompileTime> H_old;
  Eigen::Matrix<double, N_MEAS, 1> r_old;
  Eigen::Matrix<double, N_MEAS, N_MEAS> R;

  H_old.setZero();
  R.setZero();

  // get measurements
  z_p_ = Eigen::Matrix<double, 3, 1>(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  z_q_ = Eigen::Quaternion<double>(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);

  // take covariance from sensor
  R.block<6, 6> (0, 0) = Eigen::Matrix<double, 6, 6>(&msg->pose.covariance[0]);
  //clear cross-correlations between q and p
  R.block<3, 3> (0, 3) = Eigen::Matrix<double, 3, 3>::Zero();
  R.block<3, 3> (3, 0) = Eigen::Matrix<double, 3, 3>::Zero();
  R(6, 6) = 1e-6; // q_vw yaw-measurement noise

  /*************************************************************************************/
  // use this if your pose sensor is ethzasl_ptam (www.ros.org/wiki/ethzasl_ptam)
  // ethzasl_ptam publishes the camera pose as the world seen from the camera
  if (!measurement_world_sensor_)
  {
    Eigen::Matrix<double, 3, 3> C_zq = z_q_.toRotationMatrix();
    z_q_ = z_q_.conjugate();
    z_p_ = -C_zq.transpose() * z_p_;

    Eigen::Matrix<double, 6, 6> C_cov(Eigen::Matrix<double, 6, 6>::Zero());
    C_cov.block<3, 3> (0, 0) = C_zq;
    C_cov.block<3, 3> (3, 3) = C_zq;

    R.block<6, 6> (0, 0) = C_cov.transpose() * R.block<6, 6> (0, 0) * C_cov;
  }
  /*************************************************************************************/

  //  alternatively take fix covariance from reconfigure GUI
  if (use_fixed_covariance_)
  {
    const double s_zp = n_zp_ * n_zp_;
    const double s_zq = n_zq_ * n_zq_;
    R = (Eigen::Matrix<double, N_MEAS, 1>() << s_zp, s_zp, s_zp, s_zq, s_zq, s_zq, 1e-6).finished().asDiagonal();
  }

  // feedback for init case
  measurements->p_vc_ = z_p_;
  measurements->q_cv_ = z_q_;

  unsigned char idx = measurements->msf_core_.getClosestState(&state_old_o, time_old);
  if (state_old_o.time_ == -1)
    return; // // early abort // //


  const msf_core::EKFState& state_old = state_old_o;
  // get rotation matrices
  Eigen::Matrix<double, 3, 3> C_wv = state_old.get<msf_core::q_wv_>().conjugate().toRotationMatrix();
  Eigen::Matrix<double, 3, 3> C_q = state_old.get<msf_core::q_>().conjugate().toRotationMatrix();
  Eigen::Matrix<double, 3, 3> C_ci = state_old.get<msf_core::q_ci_>().conjugate().toRotationMatrix();

  // preprocess for elements in H matrix
  Eigen::Matrix<double, 3, 1> vecold;
  vecold = (state_old.get<msf_core::p_>() + C_q.transpose() * state_old.get<msf_core::p_ci_>()) * state_old.get<msf_core::L_>();
  Eigen::Matrix<double, 3, 3> skewold = skew(vecold);

  Eigen::Matrix<double, 3, 3> pci_sk = skew(state_old.get<msf_core::p_ci_>());

  // construct H matrix using H-blockx :-)
  // position:
  H_old.block<3, 3> (0, 0) = C_wv.transpose() * state_old.get<msf_core::L_>()(0); // p
  H_old.block<3, 3> (0, 6) = -C_wv.transpose() * C_q.transpose() * pci_sk * state_old.get<msf_core::L_>()(0); // q
  H_old.block<3, 1> (0, 15) = C_wv.transpose() * C_q.transpose() * state_old.get<msf_core::p_ci_>() + C_wv.transpose() * state_old.get<msf_core::p_>(); // L
  H_old.block<3, 3> (0, 16) = -C_wv.transpose() * skewold; // q_wv
  H_old.block<3, 3> (0, 22) = C_wv.transpose() * C_q.transpose() * state_old.get<msf_core::L_>()(0); //p_ci

  // attitude
  H_old.block<3, 3> (3, 6) = C_ci; // q
  H_old.block<3, 3> (3, 16) = C_ci * C_q; // q_wv
  H_old.block<3, 3> (3, 19) = Eigen::Matrix<double, 3, 3>::Identity(); //q_ci
  H_old(6, 18) = 1.0; // fix vision world yaw drift because unobservable otherwise (see PhD Thesis)

  // construct residuals
  // position
  r_old.block<3, 1> (0, 0) = z_p_ - C_wv.transpose() * (state_old.get<msf_core::p_>() + C_q.transpose() * state_old.get<msf_core::p_ci_>()) * state_old.get<msf_core::L_>();
  // attitude
  Eigen::Quaternion<double> q_err;
  q_err = (state_old.get<msf_core::q_wv_>() * state_old.get<msf_core::q_>() * state_old.get<msf_core::q_ci_>()).conjugate() * z_q_;
  r_old.block<3, 1> (3, 0) = q_err.vec() / q_err.w() * 2;
  // vision world yaw drift
  q_err = state_old.get<msf_core::q_wv_>();
  r_old(6, 0) = -2 * (q_err.w() * q_err.z() + q_err.x() * q_err.y()) / (1 - 2 * (q_err.y() * q_err.y() + q_err.z() * q_err.z()));

  // call update step in core class
  measurements->msf_core_.applyMeasurement(idx, H_old, r_old, R);
}
