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

#include "position_sensor.h"
#include <ssf_core/eigen_utils.h>

#define N_MEAS 9 // measurement size
PositionSensorHandler::PositionSensorHandler(ssf_core::Measurements* meas) :
  MeasurementHandler(meas)
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

void PositionSensorHandler::subscribe()
{
	ros::NodeHandle nh("ssf_core");
	subMeasurement_ = nh.subscribe("position_measurement", 1, &PositionSensorHandler::measurementCallback, this);

	measurements->ssf_core_.registerCallback(&PositionSensorHandler::noiseConfig, this);

	nh.param("meas_noise1",n_zp_,0.0001);	// default position noise for laser tracker total station

}

void PositionSensorHandler::noiseConfig(ssf_core::SSF_CoreConfig& config, uint32_t level)
{
	//	if(level & ssf_core::SSF_Core_MISC)
	//	{
	this->n_zp_ = config.meas_noise1;

	//	}
}

void PositionSensorHandler::measurementCallback(const ssf_updates::PositionWithCovarianceStampedConstPtr & msg)
{
	//	ROS_INFO_STREAM("measurement received \n"
	//					<< "type is: " << typeid(msg).name());

	// init variables
	ssf_core::State state_old;
	ros::Time time_old = msg->header.stamp;
	Eigen::Matrix<double,N_MEAS,N_STATE>H_old;
	Eigen::Matrix<double, N_MEAS, 1> r_old;
	Eigen::Matrix<double,N_MEAS,N_MEAS> R;

	H_old.setZero();
	R.setZero();

	// get measurements
	z_p_ = Eigen::Matrix<double,3,1>(msg->position.x, msg->position.y, msg->position.z);


	if (!use_fixed_covariance_)  // take covariance from sensor
	{
		R.block(0,0,3,3) = Eigen::Matrix<double,3,3>(&msg->covariance[0]);
		Eigen::Matrix<double,6,1> buffvec = Eigen::Matrix<double,6,1>::Constant(1e-6);
		R.block(3,3,6,6) = buffvec.asDiagonal(); // measurement noise for q_vw, q_ci
	}
	else  // alternatively take fix covariance from reconfigure GUI
	{
		const double s_zp = n_zp_ * n_zp_;
		R = (Eigen::Matrix<double, N_MEAS, 1>() << s_zp, s_zp, s_zp, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6).finished().asDiagonal();
	}

	// feedback for init case
	measurements->p_vc_ = z_p_;

	unsigned char idx = measurements->ssf_core_.getClosestState(&state_old,time_old,0);
	if (state_old.time_ == -1)
		return;	// // early abort // //

	// get rotation matrices
	Eigen::Matrix<double,3,3> C_wv = state_old.q_wv_.conjugate().toRotationMatrix();
	Eigen::Matrix<double,3,3> C_q = state_old.q_.conjugate().toRotationMatrix();
	Eigen::Matrix<double,3,3> C_ci = state_old.q_ci_.conjugate().toRotationMatrix();

	// preprocess for elements in H matrix
	Eigen::Matrix<double,3,1> vecold;
	vecold = (state_old.p_+C_q.transpose()*state_old.p_ci_)*state_old.L_;
	Eigen::Matrix<double,3,3> skewold = skew(vecold);

	Eigen::Matrix<double,3,3> pci_sk = skew(state_old.p_ci_);

	// construct H matrix using H-blockx :-)
	// position
	H_old.block<3,3>(0,0) = C_wv.transpose()*state_old.L_; // p
	H_old.block<3,3>(0,6) = -C_wv.transpose()*C_q.transpose()*pci_sk*state_old.L_; // q
	H_old.block<3,1>(0,15) = C_wv.transpose()*C_q.transpose()*state_old.p_ci_ + C_wv.transpose()*state_old.p_; // L
	H_old.block<3,3>(0,16) = -C_wv.transpose()*skewold; // q_wv
	H_old.block<3,3>(0,22) = C_wv.transpose()*C_q.transpose()*state_old.L_;	// use "camera"-IMU distance p_ci state here as position_sensor-IMU distance
	H_old.block<3,3>(3,16) = Eigen::Matrix<double,3,3>::Identity();	// fix vision world drift q_wv since it does not exist here
	H_old.block<3,3>(6,19) = Eigen::Matrix<double,3,3>::Identity();	// fix "camera"-IMU drift q_ci since it does not exist here

	// construct residuals
	// position
	r_old.block<3,1>(0,0) = z_p_ - C_wv.transpose()*(state_old.p_ + C_q.transpose()*state_old.p_ci_)*state_old.L_;
	// vision world drift q_wv
	r_old.block<3,1>(3,0) = -state_old.q_wv_.vec()/state_old.q_wv_.w()*2;
	// "camera"-IMU drift q_ci
	r_old.block<3,1>(6,0) = -state_old.q_ci_.vec()/state_old.q_ci_.w()*2;

	// call update step in core class
	measurements->ssf_core_.applyMeasurement(idx,H_old,r_old,R);
}
