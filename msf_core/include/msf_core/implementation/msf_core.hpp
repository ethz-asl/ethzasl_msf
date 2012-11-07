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

#include "calcQCore.h"
#include <msf_core/eigen_utils.h>
#include <msf_core/msf_core.hpp>

namespace msf_core
{

MSF_Core::MSF_Core(boost::shared_ptr<UserDefinedCalculations> usercalc)
{
	initialized_ = false;
	predictionMade_ = false;

	/// ros stuff
	ros::NodeHandle nh("MSF_Core");
	ros::NodeHandle pnh("~");

	pubState_ = nh.advertise<sensor_fusion_comm::DoubleArrayStamped> ("state_out", 1);
	pubCorrect_ = nh.advertise<sensor_fusion_comm::ExtEkf> ("correction", 1);
	pubPose_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped> ("pose", 1);
	pubPoseCrtl_ = nh.advertise<sensor_fusion_comm::ExtState> ("ext_state", 1);
	msgState_.data.resize(nFullState_, 0);

	subImu_ = nh.subscribe("imu_state_input", 1 /*N_STATE_BUFFER*/, &MSF_Core::imuCallback, this);
	subState_ = nh.subscribe("hl_state_input", 1 /*N_STATE_BUFFER*/, &MSF_Core::stateCallback, this);

	msgCorrect_.state.resize(HLI_EKF_STATE_SIZE, 0);
	hl_state_buf_.state.resize(HLI_EKF_STATE_SIZE, 0);

	qvw_inittimer_ = 1;

	pnh.param("data_playback", data_playback_, false);

	usercalc_ = usercalc;

	idx_state_ = 0;
	idx_P_ = 0;
	idx_time_ = 0;
}


MSF_Core::~MSF_Core()
{
}

void MSF_Core::initialize(const Eigen::Matrix<double, 3, 1> & p, const Eigen::Matrix<double, 3, 1> & v,
		const Eigen::Quaternion<double> & q, const Eigen::Matrix<double, 3, 1> & b_w,
		const Eigen::Matrix<double, 3, 1> & b_a, const double & L,
		const Eigen::Quaternion<double> & q_wv, const Eigen::Matrix<double, nErrorStatesAtCompileTime, nErrorStatesAtCompileTime> & P,
		const Eigen::Matrix<double, 3, 1> & w_m, const Eigen::Matrix<double, 3, 1> & a_m,
		const Eigen::Matrix<double, 3, 1> & g, const Eigen::Quaternion<double> & q_ci,
		const Eigen::Matrix<double, 3, 1> & p_ci)
		{
	initialized_ = false;
	predictionMade_ = false;
	qvw_inittimer_ = 1;

	// init state buffer
	for (int i = 0; i < N_STATE_BUFFER; i++)
	{
		StateBuffer_[i].reset();
	}

	idx_state_ = 0;
	idx_P_ = 0;
	idx_time_ = 0;

	msf_core::EKFState & state = StateBuffer_[idx_state_];

	this->usercalc_->initializeFirstState(state);

	state.q_int_ = state.get<msf_core::q_wv_>().state_;
	state.w_m_ = w_m;
	state.a_m_ = a_m;
	state.time_ = ros::Time::now().toSec();

	if (P.maxCoeff() == 0 && P.minCoeff() == 0){
		this->usercalc_->setP(StateBuffer_[idx_P_].P_);
	}else{
		StateBuffer_[idx_P_].P_ = P;
	}
	// constants
	g_ = g;

	// buffer for vision failure check
	qvw_inittimer_ = 1;
	qbuff_ = Eigen::Matrix<double, nBuff_, 4>::Constant(0);

	// init external propagation
	msgCorrect_.header.stamp = ros::Time::now();
	msgCorrect_.header.seq = 0;
	msgCorrect_.angular_velocity.x = 0;
	msgCorrect_.angular_velocity.y = 0;
	msgCorrect_.angular_velocity.z = 0;
	msgCorrect_.linear_acceleration.x = 0;
	msgCorrect_.linear_acceleration.y = 0;
	msgCorrect_.linear_acceleration.z = 0;
	msgCorrect_.state[0] = state.get<msf_core::p_>().state_(0);
	msgCorrect_.state[1] = state.get<msf_core::p_>().state_(1);
	msgCorrect_.state[2] = state.get<msf_core::p_>().state_(2);
	msgCorrect_.state[3] = state.get<msf_core::v_>().state_(0);
	msgCorrect_.state[4] = state.get<msf_core::v_>().state_(1);
	msgCorrect_.state[5] = state.get<msf_core::v_>().state_(2);
	msgCorrect_.state[6] = state.get<msf_core::q_>().state_.w();
	msgCorrect_.state[7] = state.get<msf_core::q_>().state_.x();
	msgCorrect_.state[8] = state.get<msf_core::q_>().state_.y();
	msgCorrect_.state[9] = state.get<msf_core::q_>().state_.z();
	msgCorrect_.state[10] = state.get<msf_core::b_w_>().state_(0);
	msgCorrect_.state[11] = state.get<msf_core::b_w_>().state_(1);
	msgCorrect_.state[12] = state.get<msf_core::b_w_>().state_(2);
	msgCorrect_.state[13] = state.get<msf_core::b_a_>().state_(0);
	msgCorrect_.state[14] = state.get<msf_core::b_a_>().state_(1);
	msgCorrect_.state[15] = state.get<msf_core::b_a_>().state_(2);
	msgCorrect_.flag = sensor_fusion_comm::ExtEkf::initialization;
	pubCorrect_.publish(msgCorrect_);

	// increase state pointers
	idx_state_++;
	idx_P_++;

	initialized_ = true;
		}



void MSF_Core::imuCallback(const sensor_msgs::ImuConstPtr & msg)
{

	if (!initialized_)
		return; // // early abort // //

	StateBuffer_[idx_state_].time_ = msg->header.stamp.toSec();

	static int seq = 0;

	// get inputs
	StateBuffer_[idx_state_].a_m_ << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
	StateBuffer_[idx_state_].w_m_ << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;

	// remove acc spikes (TODO: find a cleaner way to do this)
	static Eigen::Matrix<double, 3, 1> last_am = Eigen::Matrix<double, 3, 1>(0, 0, 0);
	if (StateBuffer_[idx_state_].a_m_.norm() > 50)
		StateBuffer_[idx_state_].a_m_ = last_am;
	else
		last_am = StateBuffer_[idx_state_].a_m_;

	if (!predictionMade_)
	{
		if (fabs(StateBuffer_[(unsigned char)(idx_state_)].time_ - StateBuffer_[(unsigned char)(idx_state_ - 1)].time_) > 5)
		{
			ROS_WARN_STREAM_THROTTLE(2, "large time-gap re-initializing to last state\n");
			StateBuffer_[(unsigned char)(idx_state_ - 1)].time_ = StateBuffer_[(idx_state_)].time_;
			return; // // early abort // // (if timegap too big)
		}
	}

	propagateState(StateBuffer_[idx_state_].time_ - StateBuffer_[(unsigned char)(idx_state_ - 1)].time_);
	predictProcessCovariance(StateBuffer_[idx_P_].time_ - StateBuffer_[(unsigned char)(idx_P_ - 1)].time_);

	checkForNumeric((double*)(&StateBuffer_[idx_state_ - 1].get<msf_core::p_>().state_(0)), 3, "prediction p");

	predictionMade_ = true;

	msgPose_.header.stamp = msg->header.stamp;
	msgPose_.header.seq = msg->header.seq;

	StateBuffer_[(unsigned char)(idx_state_ - 1)].toPoseMsg(msgPose_);
	pubPose_.publish(msgPose_);

	msgPoseCtrl_.header = msgPose_.header;
	StateBuffer_[(unsigned char)(idx_state_ - 1)].toExtStateMsg(msgPoseCtrl_);
	pubPoseCrtl_.publish(msgPoseCtrl_);

	seq++;
}



void MSF_Core::stateCallback(const sensor_fusion_comm::ExtEkfConstPtr & msg)
{

	if (!initialized_)
		return; // // early abort // //

	StateBuffer_[idx_state_].time_ = msg->header.stamp.toSec();

	static int seq = 0;

	// get inputs
	StateBuffer_[idx_state_].a_m_ << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
	StateBuffer_[idx_state_].w_m_ << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;

	// remove acc spikes (TODO: find a cleaner way to do this)
	static Eigen::Matrix<double, 3, 1> last_am = Eigen::Matrix<double, 3, 1>(0, 0, 0);
	if (StateBuffer_[idx_state_].a_m_.norm() > 50)
		StateBuffer_[idx_state_].a_m_ = last_am;
	else
		last_am = StateBuffer_[idx_state_].a_m_;

	if (!predictionMade_)
	{
		if (fabs(StateBuffer_[(idx_state_)].time_ - StateBuffer_[(unsigned char)(idx_state_ - 1)].time_) > 5)
		{
			ROS_WARN_STREAM_THROTTLE(2, "large time-gap re-initializing to last state\n");
			StateBuffer_[(unsigned char)(idx_state_ - 1)].time_ = StateBuffer_[(idx_state_)].time_;
			StateBuffer_[(unsigned char)(idx_state_)].time_ = 0;
			return; // // early abort // // (if timegap too big)
		}
	}

	int32_t flag = msg->flag;
	if (data_playback_)
		flag = sensor_fusion_comm::ExtEkf::ignore_state;

	bool isnumeric = true;
	if (flag == sensor_fusion_comm::ExtEkf::current_state)
		isnumeric = checkForNumeric(&msg->state[0], 10, "before prediction p,v,q");

	isnumeric = checkForNumeric((double*)(&StateBuffer_[idx_state_].get<msf_core::p_>().state_[0]), 3, "before prediction p");

	if (flag == sensor_fusion_comm::ExtEkf::current_state && isnumeric) // state propagation is made externally, so we read the actual state
	{
		StateBuffer_[idx_state_].get<msf_core::p_>().state_ = Eigen::Matrix<double, 3, 1>(msg->state[0], msg->state[1], msg->state[2]);
		StateBuffer_[idx_state_].get<msf_core::v_>().state_ = Eigen::Matrix<double, 3, 1>(msg->state[3], msg->state[4], msg->state[5]);
		StateBuffer_[idx_state_].get<msf_core::q_>().state_ = Eigen::Quaternion<double>(msg->state[6], msg->state[7], msg->state[8], msg->state[9]);
		StateBuffer_[idx_state_].get<msf_core::q_>().state_.normalize();

		// zero props:
		StateBuffer_[idx_state_].get<msf_core::b_w_>().state_ = StateBuffer_[(unsigned char)(idx_state_ - 1)].get<msf_core::b_w_>().state_;
		StateBuffer_[idx_state_].get<msf_core::b_a_>().state_ = StateBuffer_[(unsigned char)(idx_state_ - 1)].get<msf_core::b_a_>().state_;

		//TODO change this to be generic, so move somewhere else
		StateBuffer_[idx_state_].get<msf_core::L_>().state_ = StateBuffer_[(unsigned char)(idx_state_ - 1)].get<msf_core::L_>().state_;
		StateBuffer_[idx_state_].get<msf_core::q_wv_>().state_ = StateBuffer_[(unsigned char)(idx_state_ - 1)].get<msf_core::q_wv_>().state_;
		StateBuffer_[idx_state_].get<msf_core::q_ci_>().state_ = StateBuffer_[(unsigned char)(idx_state_ - 1)].get<msf_core::q_ci_>().state_;
		StateBuffer_[idx_state_].get<msf_core::p_ci_>().state_ = StateBuffer_[(unsigned char)(idx_state_ - 1)].get<msf_core::p_ci_>().state_;
		idx_state_++;

		hl_state_buf_ = *msg;
	}
	else if (flag == sensor_fusion_comm::ExtEkf::ignore_state || !isnumeric) // otherwise let's do the state prop. here
		propagateState(StateBuffer_[idx_state_].time_ - StateBuffer_[(unsigned char)(idx_state_ - 1)].time_);

	predictProcessCovariance(StateBuffer_[idx_P_].time_ - StateBuffer_[(unsigned char)(idx_P_ - 1)].time_);

	isnumeric = checkForNumeric((double*)(&StateBuffer_[idx_state_ - 1].get<msf_core::p_>().state_[0]), 3, "prediction p");
	isnumeric = checkForNumeric((double*)(&StateBuffer_[idx_state_ - 1].P_(0)), nErrorStatesAtCompileTime * nErrorStatesAtCompileTime, "prediction done P");

	predictionMade_ = true;

	msgPose_.header.stamp = msg->header.stamp;
	msgPose_.header.seq = msg->header.seq;

	StateBuffer_[(unsigned char)(idx_state_ - 1)].toPoseMsg(msgPose_);
	pubPose_.publish(msgPose_);

	msgPoseCtrl_.header = msgPose_.header;
	StateBuffer_[(unsigned char)(idx_state_ - 1)].toExtStateMsg(msgPoseCtrl_);
	pubPoseCrtl_.publish(msgPoseCtrl_);

	seq++;
}



void MSF_Core::propagateState(const double dt)
{


	// get references to current and previous state
	EKFState& cur_state = StateBuffer_[idx_state_];
	EKFState& prev_state = StateBuffer_[(unsigned char)(idx_state_ - 1)];

	// zero props:
	cur_state.get<msf_core::b_w_>().state_ = prev_state.get<msf_core::b_w_>().state_;
	cur_state.get<msf_core::b_a_>().state_ = prev_state.get<msf_core::b_a_>().state_;
	cur_state.L_ = prev_state.L_;
	cur_state.q_wv_ = prev_state.q_wv_;
	cur_state.q_ci_ = prev_state.q_ci_;
	cur_state.p_ci_ = prev_state.p_ci_;

	//  Eigen::Quaternion<double> dq;
	Eigen::Matrix<double, 3, 1> dv;
	ConstVector3 ew = cur_state.w_m_ - cur_state.get<msf_core::b_w_>().state_;
	ConstVector3 ewold = prev_state.w_m_ - prev_state.get<msf_core::b_w_>().state_;
	ConstVector3 ea = cur_state.a_m_ - cur_state.get<msf_core::b_a_>().state_;
	ConstVector3 eaold = prev_state.a_m_ - prev_state.get<msf_core::b_a_>().state_;
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
	for (int i = 1; i < 5; i++)
	{
		div *= i;
		MatExp = MatExp + OmegaMean / div;
		OmegaMean *= OmegaMean;
	}

	// first oder quat integration matrix
	ConstMatrix4 quat_int = MatExp + 1.0 / 48.0 * (Omega * OmegaOld - OmegaOld * Omega) * dt * dt;

	// first oder quaternion integration
	cur_state.get<msf_core::q_>().state_.coeffs() = quat_int * prev_state.get<msf_core::q_>().state_.coeffs();
	cur_state.get<msf_core::q_>().state_.normalize();

	// first oder quaternion integration
	cur_state.q_int_.coeffs() = quat_int * prev_state.q_int_.coeffs();
	cur_state.q_int_.normalize();

	dv = (cur_state.get<msf_core::q_>().state_.toRotationMatrix() * ea + prev_state.get<msf_core::q_>().state_.toRotationMatrix() * eaold) / 2;
	cur_state.get<msf_core::v_>().state_ = prev_state.get<msf_core::v_>().state_ + (dv - g_) * dt;
	cur_state.get<msf_core::p_>().state_ = prev_state.get<msf_core::p_>().state_ + ((cur_state.get<msf_core::v_>().state_ + prev_state.get<msf_core::v_>().state_) / 2 * dt);
	idx_state_++;
}



void MSF_Core::predictProcessCovariance(const double dt)
{



	// noises
	ConstVector3 nav = Vector3::Constant(config_.noise_acc /* / sqrt(dt) */);
	ConstVector3 nbav = Vector3::Constant(config_.noise_accbias /* * sqrt(dt) */);

	ConstVector3 nwv = Vector3::Constant(config_.noise_gyr /* / sqrt(dt) */);
	ConstVector3 nbwv = Vector3::Constant(config_.noise_gyrbias /* * sqrt(dt) */);



	// bias corrected IMU readings
	ConstVector3 ew = StateBuffer_[idx_P_].w_m_ - StateBuffer_[idx_P_].get<msf_core::b_w_>().state_;
	ConstVector3 ea = StateBuffer_[idx_P_].a_m_ - StateBuffer_[idx_P_].get<msf_core::b_a_>().state_;

	ConstMatrix3 a_sk = skew(ea);
	ConstMatrix3 w_sk = skew(ew);
	ConstMatrix3 eye3 = Eigen::Matrix<double, 3, 3>::Identity();

	ConstMatrix3 C_eq = StateBuffer_[idx_P_].get<msf_core::q_>().state_.toRotationMatrix();

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
	Fd_.block<3, 3> (0, 3) = dt * eye3;
	Fd_.block<3, 3> (0, 6) = A;
	Fd_.block<3, 3> (0, 9) = B;
	Fd_.block<3, 3> (0, 12) = -C_eq * dt_p2_2;

	Fd_.block<3, 3> (3, 6) = C;
	Fd_.block<3, 3> (3, 9) = D;
	Fd_.block<3, 3> (3, 12) = -C_eq * dt;

	Fd_.block<3, 3> (6, 6) = E;
	Fd_.block<3, 3> (6, 9) = F;

	calc_QCore(dt, StateBuffer_[idx_P_].get<msf_core::q_>().state_, ew, ea, nav, nbav, nwv, nbwv, Qd_);

	//TODO call user Q calc

	StateBuffer_[idx_P_].P_ = Fd_ * StateBuffer_[(unsigned char)(idx_P_ - 1)].P_ * Fd_.transpose() + Qd_;

	idx_P_++;
}



bool MSF_Core::getStateAtIdx(EKFState* timestate, unsigned char idx)
{
	if (!predictionMade_)
	{
		timestate->time_ = -1;
		return false;
	}

	*timestate = StateBuffer_[idx];

	return true;
}


unsigned char MSF_Core::getClosestState(EKFState* timestate, ros::Time tstamp, double delay)
{
	if (!predictionMade_)
	{
		timestate->time_ = -1;
		return false;
	}

	unsigned char idx = (unsigned char)(idx_state_ - 1);
	double timedist = 1e100;
	double timenow = tstamp.toSec() - delay - config_.delay;

	while (fabs(timenow - StateBuffer_[idx].time_) < timedist) // timedist decreases continuously until best point reached... then rises again
	{
		timedist = fabs(timenow - StateBuffer_[idx].time_);
		idx--;
	}
	idx++; // we subtracted one too much before....

	static bool started = false;
	if (idx == 1 && !started)
		idx = 2;
	started = true;

	if (StateBuffer_[idx].time_ == 0)
		return false; // // early abort // //  not enough predictions made yet to apply measurement (too far in past)

	propPToIdx(idx); // catch up with covariance propagation if necessary

	*timestate = StateBuffer_[idx];

	return idx;
}


void MSF_Core::propPToIdx(unsigned char idx)
{
	// propagate cov matrix until idx
	if (idx<idx_state_ && (idx_P_<=idx || idx_P_>idx_state_))	//need to propagate some covs
		while (idx!=(unsigned char)(idx_P_-1))
			predictProcessCovariance(StateBuffer_[idx_P_].time_-StateBuffer_[(unsigned char)(idx_P_-1)].time_);
}


bool MSF_Core::applyCorrection(unsigned char idx_delaystate, const ErrorState & res_delayed, double fuzzythres)
{
	static int seq_m = 0;
	if (config_.fixed_scale)
	{
		correction_(15) = 0; //scale
	}

	if (config_.fixed_bias)
	{
		correction_(9) = 0; //acc bias x
		correction_(10) = 0; //acc bias y
		correction_(11) = 0; //acc bias z
		correction_(12) = 0; //gyro bias x
		correction_(13) = 0; //gyro bias y
		correction_(14) = 0; //gyro bias z
	}

	if (config_.fixed_calib)
	{
		correction_(19) = 0; //q_ic roll
		correction_(20) = 0; //q_ic pitch
		correction_(21) = 0; //q_ic yaw
		correction_(22) = 0; //p_ci x
		correction_(23) = 0; //p_ci y
		correction_(24) = 0; //p_ci z
	}

	// state update:

	// store old values in case of fuzzy tracking
	// TODO: what to do with attitude? augment measurement noise?

	EKFState & delaystate = StateBuffer_[idx_delaystate];

	const Eigen::Matrix<double, 3, 1> buff_bw = delaystate.get<msf_core::b_w_>().state_;
	const Eigen::Matrix<double, 3, 1> buff_ba = delaystate.get<msf_core::b_a_>().state_;
	const double buff_L = delaystate.get<msf_core::L_>().state_(0);
	const Eigen::Quaternion<double> buff_qwv = delaystate.get<msf_core::q_wv_>().state_;
	const Eigen::Quaternion<double> buff_qci = delaystate.get<msf_core::q_ci_>().state_;
	const Eigen::Matrix<double, 3, 1> buff_pic = delaystate.get<msf_core::p_ci_>().state_;

	delaystate.get<msf_core::p_>().state_ = delaystate.get<msf_core::p_>().state_ + correction_.block<3, 1> (0, 0);
	delaystate.get<msf_core::v_>().state_ = delaystate.get<msf_core::v_>().state_ + correction_.block<3, 1> (3, 0);
	delaystate.get<msf_core::b_w_>().state_ = delaystate.get<msf_core::b_w_>().state_ + correction_.block<3, 1> (9, 0);
	delaystate.get<msf_core::b_a_>().state_ = delaystate.get<msf_core::b_a_>().state_ + correction_.block<3, 1> (12, 0);
	delaystate.get<msf_core::L_>().state_ = delaystate.get<msf_core::L_>().state_ + correction_(15);
	if (delaystate.get<msf_core::L_>().state_(0) < 0)
	{
		ROS_WARN_STREAM_THROTTLE(1,"Negative scale detected: " << delaystate.get<msf_core::L_>().state_(0) << ". Correcting to 0.1");
		delaystate.get<msf_core::L_>().state_(0) = 0.1;
	}

	Eigen::Quaternion<double> qbuff_q = quaternionFromSmallAngle(correction_.block<3, 1> (6, 0));
	delaystate.get<msf_core::q_>().state_ = delaystate.get<msf_core::q_>().state_ * qbuff_q;
	delaystate.get<msf_core::q_>().state_.normalize();

	Eigen::Quaternion<double> qbuff_qwv = quaternionFromSmallAngle(correction_.block<3, 1> (16, 0));
	delaystate.get<msf_core::q_wv_>().state_ = delaystate.q_wv_ * qbuff_qwv;
	delaystate.get<msf_core::q_wv_>().state_.normalize();

	Eigen::Quaternion<double> qbuff_qci = quaternionFromSmallAngle(correction_.block<3, 1> (19, 0));
	delaystate.get<msf_core::q_ci_>().state_ = delaystate.get<msf_core::q_ci_ * qbuff_qci;
	delaystate.get<msf_core::q_ci_>().state_.normalize();

	delaystate.get<msf_core::p_ci_>().state_ = delaystate.get<msf_core::p_ci_>().state_ + correction_.block<3, 1> (22, 0);

	// update qbuff_ and check for fuzzy tracking
	if (qvw_inittimer_ > nBuff_)
	{
		// should be unit quaternion if no error
		Eigen::Quaternion<double> errq = delaystate.get<msf_core::q_wv_>().state_.conjugate() *
				Eigen::Quaternion<double>(
						getMedian(qbuff_.block<nBuff_, 1> (0, 3)),
						getMedian(qbuff_.block<nBuff_, 1> (0, 0)),
						getMedian(qbuff_.block<nBuff_, 1> (0, 1)),
						getMedian(qbuff_.block<nBuff_, 1> (0, 2))
				);

		if (std::max(errq.vec().maxCoeff(), -errq.vec().minCoeff()) / fabs(errq.w()) * 2 > fuzzythres) // fuzzy tracking (small angle approx)
		{
			ROS_WARN_STREAM_THROTTLE(1,"fuzzy tracking triggered: " << std::max(errq.vec().maxCoeff(), -errq.vec().minCoeff())/fabs(errq.w())*2 << " limit: " << fuzzythres <<"\n");

			//state_.q_ = buff_q;
			delaystate.b_w_ = buff_bw;
			delaystate.b_a_ = buff_ba;
			delaystate.L_ = buff_L;
			delaystate.q_wv_ = buff_qwv;
			delaystate.q_ci_ = buff_qci;
			delaystate.p_ci_ = buff_pic;
			correction_.block<16, 1> (9, 0) = Eigen::Matrix<double, 16, 1>::Zero();
			qbuff_q.setIdentity();
			qbuff_qwv.setIdentity();
			qbuff_qci.setIdentity();
		}
		else // if tracking ok: update mean and 3sigma of past N q_vw's
		{
			qbuff_.block<1, 4> (qvw_inittimer_ - nBuff_ - 1, 0) = Eigen::Matrix<double, 1, 4>(delaystate.q_wv_.coeffs());
			qvw_inittimer_ = (qvw_inittimer_) % nBuff_ + nBuff_ + 1;
		}
	}
	else // at beginning get mean and 3sigma of past N q_vw's
	{
		qbuff_.block<1, 4> (qvw_inittimer_ - 1, 0) = Eigen::Matrix<double, 1, 4>(delaystate.q_wv_.coeffs());
		qvw_inittimer_++;
	}

	// idx fiddeling to ensure correct update until now from the past
	idx_time_ = idx_state_;
	idx_state_ = idx_delaystate + 1;
	idx_P_ = idx_delaystate + 1;

	// propagate state matrix until now
	while (idx_state_ != idx_time_)
		propagateState(StateBuffer_[idx_state_].time_ - StateBuffer_[(unsigned char)(idx_state_ - 1)].time_);

	checkForNumeric(&correction_[0], HLI_EKF_STATE_SIZE, "update");

	// publish correction for external propagation
	msgCorrect_.header.stamp = ros::Time::now();
	msgCorrect_.header.seq = seq_m;
	msgCorrect_.angular_velocity.x = 0;
	msgCorrect_.angular_velocity.y = 0;
	msgCorrect_.angular_velocity.z = 0;
	msgCorrect_.linear_acceleration.x = 0;
	msgCorrect_.linear_acceleration.y = 0;
	msgCorrect_.linear_acceleration.z = 0;

	const unsigned char idx = (unsigned char)(idx_state_ - 1);
	msgCorrect_.state[0] = StateBuffer_[idx].get<msf::core<p_>().state_[0] - hl_state_buf_.state[0];
	msgCorrect_.state[1] = StateBuffer_[idx].get<msf::core<p_>().state_[1] - hl_state_buf_.state[1];
	msgCorrect_.state[2] = StateBuffer_[idx].get<msf::core<p_>().state_[2] - hl_state_buf_.state[2];
	msgCorrect_.state[3] = StateBuffer_[idx].get<msf::core<v_>().state_[0] - hl_state_buf_.state[3];
	msgCorrect_.state[4] = StateBuffer_[idx].get<msf::core<v_>().state_[1] - hl_state_buf_.state[4];
	msgCorrect_.state[5] = StateBuffer_[idx].get<msf::core<v_>().state_[2] - hl_state_buf_.state[5];

	Eigen::Quaterniond hl_q(hl_state_buf_.state[6], hl_state_buf_.state[7], hl_state_buf_.state[8], hl_state_buf_.state[9]);
	qbuff_q = hl_q.inverse() * StateBuffer_[idx].get<msf::core<q_>().state_;
	msgCorrect_.state[6] = qbuff_q.w();
	msgCorrect_.state[7] = qbuff_q.x();
	msgCorrect_.state[8] = qbuff_q.y();
	msgCorrect_.state[9] = qbuff_q.z();

	msgCorrect_.state[10] = StateBuffer_[idx].get<msf::core<b_w_>().state_[0] - hl_state_buf_.state[10];
	msgCorrect_.state[11] = StateBuffer_[idx].get<msf::core<b_w_>().state_[1] - hl_state_buf_.state[11];
	msgCorrect_.state[12] = StateBuffer_[idx].get<msf::core<b_w_>().state_[2] - hl_state_buf_.state[12];
	msgCorrect_.state[13] = StateBuffer_[idx].get<msf::core<b_a_>().state_[0] - hl_state_buf_.state[13];
	msgCorrect_.state[14] = StateBuffer_[idx].get<msf::core<b_a_>().state_[1] - hl_state_buf_.state[14];
	msgCorrect_.state[15] = StateBuffer_[idx].get<msf::core<b_a_>().state_[2] - hl_state_buf_.state[15];

	msgCorrect_.flag = sensor_fusion_comm::ExtEkf::state_correction;
	pubCorrect_.publish(msgCorrect_);

	// publish state
	msgState_.header = msgCorrect_.header;
	StateBuffer_[idx].toStateMsg(msgState_);
	pubState_.publish(msgState_);
	seq_m++;

	return 1;
}


double MSF_Core::getMedian(const Eigen::Matrix<double, nBuff_, 1> & data)
{
	std::vector<double> mediandistvec;
	mediandistvec.reserve(nBuff_);
	for (int i = 0; i < nBuff_; ++i)
		mediandistvec.push_back(data(i));

	if (mediandistvec.size() > 0)
	{
		std::vector<double>::iterator first = mediandistvec.begin();
		std::vector<double>::iterator last = mediandistvec.end();
		std::vector<double>::iterator middle = first + std::floor((last - first) / 2);
		std::nth_element(first, middle, last); // can specify comparator as optional 4th arg
		return *middle;
	}
	else
		return 0;
}

}; // end namespace
