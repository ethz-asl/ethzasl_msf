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
#include <msf_core/msf_sensormanager.hpp>
#include <msf_core/msf_tools.hpp>

namespace msf_core
{

MSF_Core::MSF_Core(MSF_SensorManager* usercalc)
{
	initialized_ = false;
	predictionMade_ = false;


	//TODO later: move all this to the external file and derive from this class. We could by this allow compilation on platforms withour ROS
	/// ros stuff
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

	usercalc_ = usercalc; //the interface for the user to customize EKF interna, do NOT USE THIS POINTER INSIDE THE CTOR

	// buffer for vision failure check
if(indexOfStateWithoutTemporalDrift != -1){
	nontemporaldrifting_inittimer_ = 1;

	qbuff_ = Eigen::Matrix<double, nBuff_, qbuffRowsAtCompiletime>::Constant(0);
}
	idx_state_ = 0;
	idx_P_ = 0;
	idx_time_ = 0;
}


MSF_Core::~MSF_Core()
{
}


void MSF_Core::initialize(const Eigen::Matrix<double, 3, 1> & p, const Eigen::Matrix<double, 3, 1> & v,
		const Eigen::Quaternion<double> & q, const Eigen::Matrix<double, 3, 1> & b_w,
		const Eigen::Matrix<double, 3, 1> & b_a, const Eigen::Matrix<double, nErrorStatesAtCompileTime, nErrorStatesAtCompileTime> & P,
		const Eigen::Matrix<double, 3, 1> & w_m, const Eigen::Matrix<double, 3, 1> & a_m,
		const Eigen::Matrix<double, 3, 1> & g)
{
	initialized_ = false;
	predictionMade_ = false;

	// init state buffer
	for (int i = 0; i < N_STATE_BUFFER; i++)
	{
		//slynen{
		StateBuffer_[i].reset(usercalc_);
		//}
	}

	idx_state_ = 0;
	idx_P_ = 0;
	idx_time_ = 0;

	msf_core::EKFState & state = StateBuffer_[idx_state_];
	//slynen{
	state.get<msf_core::p_>() = p;
	state.get<msf_core::v_>() = v;
	state.get<msf_core::q_>() = q;
	state.get<msf_core::b_w_>() = b_w;
	state.get<msf_core::b_a_>() = b_a;


	this->usercalc_->initState(state); //ask the user to provide sensible data for the first state
	//}
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

	// init external propagation
	msgCorrect_.header.stamp = ros::Time::now();
	msgCorrect_.header.seq = 0;
	msgCorrect_.angular_velocity.x = 0;
	msgCorrect_.angular_velocity.y = 0;
	msgCorrect_.angular_velocity.z = 0;
	msgCorrect_.linear_acceleration.x = 0;
	msgCorrect_.linear_acceleration.y = 0;
	msgCorrect_.linear_acceleration.z = 0;

	//slynen{
	msgCorrect_.state.resize(nStatesAtCompileTime); //make sure this is correctly sized
	boost::fusion::for_each(
			state.statevars_,
			msf_tmp::CoreStatetoDoubleArray<std::vector<float>, EKFState::stateVector_T >(msgCorrect_.state)
	);
	//}

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

	checkForNumeric((double*)(&StateBuffer_[idx_state_ - 1].get<msf_core::p_>()(0)), 3, "prediction p");

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

/// main update routine called by a given sensor
template<class H_type, class Res_type, class R_type>
bool MSF_Core::applyMeasurement(unsigned char idx_delaystate, const Eigen::MatrixBase<H_type>& H_delayed,
		const Eigen::MatrixBase<Res_type> & res_delayed, const Eigen::MatrixBase<R_type>& R_delayed,
		double fuzzythres)
{
	EIGEN_STATIC_ASSERT_FIXED_SIZE(H_type);
	EIGEN_STATIC_ASSERT_FIXED_SIZE(R_type);

	// get measurements
	if (!predictionMade_)
		return false;

	// make sure we have correctly propagated cov until idx_delaystate
	propPToIdx(idx_delaystate);

	R_type S;
	Eigen::Matrix<double, nErrorStatesAtCompileTime, R_type::RowsAtCompileTime> K;
	ErrorStateCov & P = StateBuffer_[idx_delaystate].P_;

	S = H_delayed * StateBuffer_[idx_delaystate].P_ * H_delayed.transpose() + R_delayed;
	K = P * H_delayed.transpose() * S.inverse();

	correction_ = K * res_delayed;
	const ErrorStateCov KH = (ErrorStateCov::Identity() - K * H_delayed);
	P = KH * P * KH.transpose() + K * R_delayed * K.transpose();

	// make sure P stays symmetric
	P = 0.5 * (P + P.transpose());

	return applyCorrection(idx_delaystate, correction_, fuzzythres);
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

	isnumeric = checkForNumeric((double*)(&StateBuffer_[idx_state_].get<msf_core::p_>()[0]), 3, "before prediction p");

	if (flag == sensor_fusion_comm::ExtEkf::current_state && isnumeric) // state propagation is made externally, so we read the actual state
	{
		StateBuffer_[idx_state_].get<msf_core::p_>() = Eigen::Matrix<double, 3, 1>(msg->state[0], msg->state[1], msg->state[2]);
		StateBuffer_[idx_state_].get<msf_core::v_>() = Eigen::Matrix<double, 3, 1>(msg->state[3], msg->state[4], msg->state[5]);
		StateBuffer_[idx_state_].get<msf_core::q_>() = Eigen::Quaternion<double>(msg->state[6], msg->state[7], msg->state[8], msg->state[9]);
		StateBuffer_[idx_state_].get<msf_core::q_>().normalize();

		// zero props:
		//slynen{ copy non propagation states from last state
		boost::fusion::for_each(
				StateBuffer_[idx_state_].statevars_,
				msf_tmp::copyNonPropagationStates<EKFState>(StateBuffer_[(unsigned char)(idx_state_ - 1)])
		);
		//}

		idx_state_++;

		hl_state_buf_ = *msg;
	}
	else if (flag == sensor_fusion_comm::ExtEkf::ignore_state || !isnumeric) // otherwise let's do the state prop. here
		propagateState(StateBuffer_[idx_state_].time_ - StateBuffer_[(unsigned char)(idx_state_ - 1)].time_);

	predictProcessCovariance(StateBuffer_[idx_P_].time_ - StateBuffer_[(unsigned char)(idx_P_ - 1)].time_);

	isnumeric = checkForNumeric((double*)(&StateBuffer_[idx_state_ - 1].get<msf_core::p_>()[0]), 3, "prediction p");
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
	//slynen{
	//copy constant for non propagated states
	boost::fusion::for_each(
			cur_state.statevars_,
			msf_tmp::copyNonPropagationStates<EKFState>(prev_state)
	);
	//}

	//  Eigen::Quaternion<double> dq;
	Eigen::Matrix<double, 3, 1> dv;
	ConstVector3 ew = cur_state.w_m_ - cur_state.get<msf_core::b_w_>();
	ConstVector3 ewold = prev_state.w_m_ - prev_state.get<msf_core::b_w_>();
	ConstVector3 ea = cur_state.a_m_ - cur_state.get<msf_core::b_a_>();
	ConstVector3 eaold = prev_state.a_m_ - prev_state.get<msf_core::b_a_>();
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
	cur_state.get<msf_core::q_>().coeffs() = quat_int * prev_state.get<msf_core::q_>().coeffs();
	cur_state.get<msf_core::q_>().normalize();

	dv = (cur_state.get<msf_core::q_>().toRotationMatrix() * ea + prev_state.get<msf_core::q_>().toRotationMatrix() * eaold) / 2;
	cur_state.get<msf_core::v_>() = prev_state.get<msf_core::v_>() + (dv - g_) * dt;
	cur_state.get<msf_core::p_>() = prev_state.get<msf_core::p_>() + ((cur_state.get<msf_core::v_>() + prev_state.get<msf_core::v_>()) / 2 * dt);
	idx_state_++;
}



void MSF_Core::predictProcessCovariance(const double dt)
{
	// noises
	ConstVector3 nav = Vector3::Constant(usercalc_->getParam_noise_acc() /* / sqrt(dt) */);
	ConstVector3 nbav = Vector3::Constant(usercalc_->getParam_noise_accbias() /* * sqrt(dt) */);

	ConstVector3 nwv = Vector3::Constant(usercalc_->getParam_noise_gyr() /* / sqrt(dt) */);
	ConstVector3 nbwv = Vector3::Constant(usercalc_->getParam_noise_gyrbias() /* * sqrt(dt) */);

	// bias corrected IMU readings
	ConstVector3 ew = StateBuffer_[idx_P_].w_m_ - StateBuffer_[idx_P_].get<msf_core::b_w_>();
	ConstVector3 ea = StateBuffer_[idx_P_].a_m_ - StateBuffer_[idx_P_].get<msf_core::b_a_>();

	ConstMatrix3 a_sk = skew(ea);
	ConstMatrix3 w_sk = skew(ew);
	ConstMatrix3 eye3 = Eigen::Matrix<double, 3, 3>::Identity();

	ConstMatrix3 C_eq = StateBuffer_[idx_P_].get<msf_core::q_>().toRotationMatrix();

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

	calc_QCore(dt, StateBuffer_[idx_P_].get<msf_core::q_>(), ew, ea, nav, nbav, nwv, nbwv, Qd_);

	//call user Q calc to fill in the blocks of auxiliary states
	usercalc_->calculateQAuxiliaryStates(StateBuffer_[idx_P_], dt);

	//now copy the userdefined blocks to Qd
	boost::fusion::for_each(
			StateBuffer_[idx_P_].statevars_,
			msf_tmp::copyQBlocksFromAuxiliaryStatesToQ<EKFState::stateVector_T>(Qd_)
	);

	//TODO later: optimize here multiplication of F blockwise, using the fact that aux states have no entries outside their block
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
	double timenow = tstamp.toSec() - delay - usercalc_->getParam_delay();

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

	//slynen{
	//give the user the possibility to fix some states
	usercalc_->augmentCorrectionVector(correction_);

	//now augment core states
	if (usercalc_->getParam_fixed_bias())
	{
		typedef typename msf_tmp::getEnumStateType<msf_core::EKFState::stateVector_T, msf_core::b_a_>::value b_a_type;
		typedef typename msf_tmp::getEnumStateType<msf_core::EKFState::stateVector_T, msf_core::b_w_>::value b_w_type;

		enum{
			indexOfState_b_a = msf_tmp::getStartIndex<msf_core::EKFState::stateVector_T, b_a_type, msf_tmp::CorrectionStateLengthForType>::value,
			indexOfState_b_w = msf_tmp::getStartIndex<msf_core::EKFState::stateVector_T, b_w_type, msf_tmp::CorrectionStateLengthForType>::value
		};

		//		msf_tmp::echoCompileTimeConstant<indexOfState_b_a>();
		//		msf_tmp::echoCompileTimeConstant<indexOfState_b_w>();

		BOOST_STATIC_ASSERT_MSG(static_cast<int>(indexOfState_b_w)==9, "The index of the state b_w in the correction vector differs from the expected value");
		BOOST_STATIC_ASSERT_MSG(static_cast<int>(indexOfState_b_a)==12, "The index of the state b_a in the correction vector differs from the expected value");

		for(int i = 0 ; i < msf_tmp::StripConstReference<b_a_type>::result_t::sizeInCorrection_ ; ++i){
			correction_(indexOfState_b_a + i) = 0; //acc bias x,y,z
		}
		for(int i = 0 ; i < msf_tmp::StripConstReference<b_w_type>::result_t::sizeInCorrection_ ; ++i){
			correction_(indexOfState_b_w + i) = 0; //gyro bias x,y,z
		}
	}
	//}

	// state update:

	// store old values in case of fuzzy tracking
	// TODO: sweiss what to do with attitude? augment measurement noise?

	EKFState & delaystate = StateBuffer_[idx_delaystate];

	EKFState buffstate = delaystate;

	//slynen{ call correction function for every state

	Eigen::Quaternion<double> qbuff_q = quaternionFromSmallAngle(correction_.block<3, 1> (6, 0));

	delaystate.correct(correction_);

	//}

	//slynen{ //allow the user to sanity check the new state
	usercalc_->sanityCheckCorrection(delaystate, buffstate, correction_);

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
		Eigen::Quaternion<double> errq = const_cast<const EKFState&>(delaystate).get<indexOfStateWithoutTemporalDrift>().conjugate() *
				Eigen::Quaternion<double>(
						getMedian(qbuff_.block<nBuff_, 1> (0, 3)),
						getMedian(qbuff_.block<nBuff_, 1> (0, 0)),
						getMedian(qbuff_.block<nBuff_, 1> (0, 1)),
						getMedian(qbuff_.block<nBuff_, 1> (0, 2))
				);

		if (std::max(errq.vec().maxCoeff(), -errq.vec().minCoeff()) / fabs(errq.w()) * 2 > fuzzythres) // fuzzy tracking (small angle approx)
		{
			ROS_WARN_STREAM_THROTTLE(1,"fuzzy tracking triggered: " << std::max(errq.vec().maxCoeff(), -errq.vec().minCoeff())/fabs(errq.w())*2 << " limit: " << fuzzythres <<"\n");

			//copy the non propagation states back from the buffer
			boost::fusion::for_each(
					delaystate.statevars_,
					msf_tmp::copyNonPropagationStates<EKFState>(buffstate)
			);

			BOOST_STATIC_ASSERT_MSG(static_cast<int>(EKFState::nPropagatedCoreErrorStatesAtCompileTime) == 9, "Assumed that nPropagatedCoreStates == 9, which is not the case");
			BOOST_STATIC_ASSERT_MSG(static_cast<int>(EKFState::nErrorStatesAtCompileTime) -
					static_cast<int>(EKFState::nPropagatedCoreErrorStatesAtCompileTime) == 16, "Assumed that nErrorStatesAtCompileTime-nPropagatedCoreStates == 16, which is not the case");

			//TODO: can be eliminated
			correction_.block<EKFState::nErrorStatesAtCompileTime-EKFState::nPropagatedCoreErrorStatesAtCompileTime, 1> (EKFState::nPropagatedCoreErrorStatesAtCompileTime, 0) =
					Eigen::Matrix<double, EKFState::nErrorStatesAtCompileTime-EKFState::nPropagatedCoreErrorStatesAtCompileTime, 1>::Zero();
			qbuff_q.setIdentity();
		}
		else // if tracking ok: update mean and 3sigma of past N q_vw's
		{
			qbuff_.block<1, 4> (nontemporaldrifting_inittimer_ - nBuff_ - 1, 0) = Eigen::Matrix<double, 1, 4>(const_cast<const EKFState&>(delaystate).get<indexOfStateWithoutTemporalDrift>().coeffs());
			nontemporaldrifting_inittimer_ = (nontemporaldrifting_inittimer_) % nBuff_ + nBuff_ + 1;
		}
	}
	else // at beginning get mean and 3sigma of past N q_vw's
	{
		qbuff_.block<1, 4> (nontemporaldrifting_inittimer_ - 1, 0) = Eigen::Matrix<double, 1, 4>(const_cast<const EKFState&>(delaystate).get<indexOfStateWithoutTemporalDrift>().coeffs());
		nontemporaldrifting_inittimer_++;
	}
} //end fuzzy tracking

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
	msgCorrect_.state[0] = StateBuffer_[idx].get<msf_core::p_>()[0] - hl_state_buf_.state[0];
	msgCorrect_.state[1] = StateBuffer_[idx].get<msf_core::p_>()[1] - hl_state_buf_.state[1];
	msgCorrect_.state[2] = StateBuffer_[idx].get<msf_core::p_>()[2] - hl_state_buf_.state[2];
	msgCorrect_.state[3] = StateBuffer_[idx].get<msf_core::v_>()[0] - hl_state_buf_.state[3];
	msgCorrect_.state[4] = StateBuffer_[idx].get<msf_core::v_>()[1] - hl_state_buf_.state[4];
	msgCorrect_.state[5] = StateBuffer_[idx].get<msf_core::v_>()[2] - hl_state_buf_.state[5];

	Eigen::Quaterniond hl_q(hl_state_buf_.state[6], hl_state_buf_.state[7], hl_state_buf_.state[8], hl_state_buf_.state[9]);
	qbuff_q = hl_q.inverse() * StateBuffer_[idx].get<msf_core::q_>();
	msgCorrect_.state[6] = qbuff_q.w();
	msgCorrect_.state[7] = qbuff_q.x();
	msgCorrect_.state[8] = qbuff_q.y();
	msgCorrect_.state[9] = qbuff_q.z();

	msgCorrect_.state[10] = StateBuffer_[idx].get<msf_core::b_w_>()[0] - hl_state_buf_.state[10];
	msgCorrect_.state[11] = StateBuffer_[idx].get<msf_core::b_w_>()[1] - hl_state_buf_.state[11];
	msgCorrect_.state[12] = StateBuffer_[idx].get<msf_core::b_w_>()[2] - hl_state_buf_.state[12];
	msgCorrect_.state[13] = StateBuffer_[idx].get<msf_core::b_a_>()[0] - hl_state_buf_.state[13];
	msgCorrect_.state[14] = StateBuffer_[idx].get<msf_core::b_a_>()[1] - hl_state_buf_.state[14];
	msgCorrect_.state[15] = StateBuffer_[idx].get<msf_core::b_a_>()[2] - hl_state_buf_.state[15];

	msgCorrect_.flag = sensor_fusion_comm::ExtEkf::state_correction;
	pubCorrect_.publish(msgCorrect_);

	// publish state
	msgState_.header = msgCorrect_.header;
	StateBuffer_[idx].toFullStateMsg(msgState_);
	pubState_.publish(msgState_);
	seq_m++;

	return 1;
}




}; // end namespace
