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

#include "viconpos_sensor.h"
#include <msf_core/eigen_utils.h>
#include <msf_core/msf_sensormanagerROS.hpp>

#define N_MEAS 9 // measurement size
PositionSensorHandler::PositionSensorHandler(msf_core::MSF_SensorManager* meas) :
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

  //initialize the measurements this handler provides
  z_p_ = Eigen::Matrix<double, 3, 1>::Constant(0);
  z_q_ = Eigen::Quaterniond::Identity();
}

void PositionSensorHandler::subscribe()
{
	ros::NodeHandle nh("msf_core");
	subMeasurement_ = nh.subscribe("position_measurement", 1, &PositionSensorHandler::measurementCallback, this);

	dynamic_cast<msf_core::MSF_SensorManagerROS*>(measurements)->registerCallback(&PositionSensorHandler::noiseConfig, this);

	nh.param("meas_noise1",n_zp_,0.0001);	// default position noise for laser tracker total station

}

void PositionSensorHandler::noiseConfig(msf_core::MSF_CoreConfig& config, uint32_t level)
{
	//	if(level & msf_core::MSF_Core_MISC)
	//	{
	this->n_zp_ = config.meas_noise1;

	//	}
}

void PositionSensorHandler::measurementCallback(const geometry_msgs::TransformStampedConstPtr & msg)
{
	//	ROS_INFO_STREAM("measurement received \n"
	//					<< "type is: " << typeid(msg).name());
	//slow down VICON
	if (msg->header.seq%5!=0)
	  return;

	// init variables
	msf_core::EKFState state_old_o;
	ros::Time time_old = msg->header.stamp;
	Eigen::Matrix<double,N_MEAS,msf_core::MSF_Core::nErrorStatesAtCompileTime> H_old;
	Eigen::Matrix<double, N_MEAS, 1> r_old;
	Eigen::Matrix<double,N_MEAS,N_MEAS> R;

	H_old.setZero();
	R.setZero();

	// get measurements
	z_p_ = Eigen::Matrix<double,3,1>(msg->transform.translation.x, msg->transform.translation.y, msg->transform.translation.z);


	if (!use_fixed_covariance_)  // take covariance from sensor
	{
//		R.block(0,0,3,3) = Eigen::Matrix<double,3,3>(&msg->covariance[0]);
//		Eigen::Matrix<double,6,1> buffvec = Eigen::Matrix<double,6,1>::Constant(1e-6);
//		R.block(3,3,6,6) = buffvec.asDiagonal(); // measurement noise for q_vw, q_ci
	}
	else  // alternatively take fix covariance from reconfigure GUI
	{
		const double s_zp = n_zp_ * n_zp_;
		R = (Eigen::Matrix<double, N_MEAS, 1>() << s_zp, s_zp, s_zp, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6).finished().asDiagonal();
	}



	unsigned char idx = measurements->msf_core_->getClosestState(&state_old_o,time_old,0);
	if (state_old_o.time_ == -1)
		return;	// // early abort // //

	const msf_core::EKFState& state_old = state_old_o;

	// get rotation matrices
	Eigen::Matrix<double,3,3> C_wv = state_old.get<msf_core::q_wv_>().conjugate().toRotationMatrix();
	Eigen::Matrix<double,3,3> C_q = state_old.get<msf_core::q_>().conjugate().toRotationMatrix();
	Eigen::Matrix<double,3,3> C_ci = state_old.get<msf_core::q_ci_>().conjugate().toRotationMatrix();

	// preprocess for elements in H matrix
	Eigen::Matrix<double,3,1> vecold;
	vecold = (state_old.get<msf_core::p_>()+C_q.transpose()*state_old.get<msf_core::p_ci_>())*state_old.get<msf_core::L_>();
	Eigen::Matrix<double,3,3> skewold = skew(vecold);

	Eigen::Matrix<double,3,3> pci_sk = skew(state_old.get<msf_core::p_ci_>());

	// construct H matrix using H-blockx :-)
	// position
	H_old.block<3,3>(0,0) = C_wv.transpose()*state_old.get<msf_core::L_>()(0); // p
	H_old.block<3,3>(0,6) = -C_wv.transpose()*C_q.transpose()*pci_sk*state_old.get<msf_core::L_>()(0); // q
	H_old.block<3,1>(0,15) = C_wv.transpose()*C_q.transpose()*state_old.get<msf_core::p_ci_>() + C_wv.transpose()*state_old.get<msf_core::p_>(); // L
	H_old.block<3,3>(0,16) = -C_wv.transpose()*skewold; // q_wv
	H_old.block<3,3>(0,22) = C_wv.transpose()*C_q.transpose()*state_old.get<msf_core::L_>()(0);	// use "camera"-IMU distance p_ci state here as position_sensor-IMU distance
	H_old.block<3,3>(3,16) = Eigen::Matrix<double,3,3>::Identity();	// fix vision world drift q_wv since it does not exist here
	H_old.block<3,3>(6,19) = Eigen::Matrix<double,3,3>::Identity();	// fix "camera"-IMU drift q_ci since it does not exist here

	// construct residuals
	// position
	r_old.block<3,1>(0,0) = z_p_ - C_wv.transpose()*(state_old.get<msf_core::p_>() + C_q.transpose()*state_old.get<msf_core::p_ci_>())*state_old.get<msf_core::L_>();
	// vision world drift q_wv
	r_old.block<3,1>(3,0) = -state_old.get<msf_core::q_wv_>().vec()/state_old.get<msf_core::q_wv_>().w()*2;
	// "camera"-IMU drift q_ci
	r_old.block<3,1>(6,0) = -state_old.get<msf_core::q_ci_>().vec()/state_old.get<msf_core::q_ci_>().w()*2;

	// call update step in core class
	measurements->msf_core_->applyMeasurement(idx,H_old,r_old,R);
}
