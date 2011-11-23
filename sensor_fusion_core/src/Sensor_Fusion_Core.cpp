/*
 * Sensor_Fusion_Core.cpp
 *
 *  Created on: Sep 30, 2010
 *      Author: sweiss
 *      NOTE: This filter uses the quaternion convention proposed by
 *      "Quaternions - proposed standard conventions" JPL, Tech. Rep.
 *      That is: -ij=ji=k, -jk=kj=i, -ki=ik=j
 *      Whereas Eigen uses internally ij=-ji=k, jk=-kj=i, ki=-ik=j
 *      This leads to some differencies wrt. the matlab version. I.e.
 *      the rotation notation (not the equations) is the other way round...
 *      So, q_vw in matlab is q_wv here.
 */

#include <sensor_fusion_core/Sensor_Fusion_Core.h>
#include "calcQ.h"

template<class T>
bool checkForNumeric(T vec, int size, const std::string & info)
{
  for (int i = 0; i < size; i++)
  {
    if (isnan(vec[i]))
    {
      ROS_ERROR_STREAM(info.c_str() << ": NAN at index " << i);
//      ROS_BREAK();
      return false;
    }
    if (isinf(vec[i]))
    {
        ROS_ERROR_STREAM(info.c_str() << ": INF at index " << i);
//      ROS_BREAK();
        return false;
    }
  }
  return true;
}

Sensor_Fusion_Core::Sensor_Fusion_Core(){
	initialized_ = false;
	predictionMade_ = false;
	fixedScale_ = false;

	// ros stuff
	ros::NodeHandle nh("sensor_fusion");

	pubState_ = nh.advertise<sensor_fusion_core::DoubleArrayStamped>("state_out", 1);
	pubCorrect_ = nh.advertise<sensor_fusion_core::ext_ekf>("correction", 1);
	pubPose_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 1);
	pubPoseCrtl_ = nh.advertise<sensor_fusion_core::ext_state>("ext_state", 1);
	msgState_.data.resize(nFullState_, 0);

	subImu_ = nh.subscribe("imu_state_input", 1 /*nStateBuffer_*/, &Sensor_Fusion_Core::imuCallback, this);
	subState_ = nh.subscribe("hl_state_input", 1 /*nStateBuffer_*/, &Sensor_Fusion_Core::stateCallback, this);

	msgCorrect_.state.resize(HLI_EKF_STATE_SIZE , 0);
	hl_state_buf_.state.resize(HLI_EKF_STATE_SIZE , 0);

	qvw_inittimer_ = 1;

	reconfServer_ = new ReconfigureServer(ros::NodeHandle("~"));
	ReconfigureServer::CallbackType f = boost::bind(&Sensor_Fusion_Core::Config, this , _1, _2);
	reconfServer_->setCallback(f);
	//register dyn config list
	registerCallback(&Sensor_Fusion_Core::DynConfig, this);
}

Sensor_Fusion_Core::~Sensor_Fusion_Core() {
	// TODO Auto-generated destructor stub
	delete reconfServer_;
}

void Sensor_Fusion_Core::initialize(Eigen::Matrix<double, 3, 1> p, Eigen::Matrix<double, 3, 1> v,Eigen::Quaternion<double> q,
		Eigen::Matrix<double, 3, 1> b_w,Eigen::Matrix<double, 3, 1> b_a,double L,Eigen::Quaternion<double> q_wv,
		Eigen::Matrix<double, nState_, nState_> P,
		Eigen::Matrix<double, 3, 1> w_m,Eigen::Matrix<double, 3, 1> a_m,
		Eigen::Matrix<double, 3, 1> g, Eigen::Quaternion<double> q_ci, Eigen::Matrix<double, 3, 1> p_ic,
		Eigen::Quaternion<double> q_mi,  Eigen::Matrix<double, 3, 1> p_ig, Eigen::Matrix<double, 3, 1> p_vw,
		double alpha, double beta)
{

//	boost::mutex::scoped_lock lock(ekf_mutex_);

	// init state buffer
	for(int i=0; i<nStateBuffer_;i++)
	{
		StateBuffer_[i].p_ = Eigen::Matrix<double, 3, 1>::Constant(0);
		StateBuffer_[i].v_ = Eigen::Matrix<double, 3, 1>::Constant(0);
		StateBuffer_[i].q_ = Eigen::Quaternion<double>(1, 0, 0, 0);
		StateBuffer_[i].b_w_ = Eigen::Matrix<double, 3, 1>::Constant(0);
		StateBuffer_[i].b_a_ = Eigen::Matrix<double, 3, 1>::Constant(0);
		StateBuffer_[i].L_ = 0;
		StateBuffer_[i].q_wv_ = Eigen::Quaternion<double>(1, 0, 0, 0);
		StateBuffer_[i].q_ci_ = Eigen::Quaternion<double>(1, 0, 0, 0);
		StateBuffer_[i].p_ic_ = Eigen::Matrix<double, 3, 1>::Constant(0);
		StateBuffer_[i].q_mi_ = Eigen::Quaternion<double>(1, 0, 0, 0);
		StateBuffer_[i].p_ig_ = Eigen::Matrix<double, 3, 1>::Constant(0);
		StateBuffer_[i].p_vw_ = Eigen::Matrix<double, 3, 1>::Constant(0);
		StateBuffer_[i].alpha_ = 0;
		StateBuffer_[i].beta_ = 0;
		StateBuffer_[i].P_ = Eigen::Matrix<double,nState_,nState_>::Constant(0);
		StateBuffer_[i].w_m_ = Eigen::Matrix<double, 3, 1>::Constant(0);
		StateBuffer_[i].q_int_ = Eigen::Quaternion<double>(1, 0, 0, 0);
		StateBuffer_[i].a_m_ = Eigen::Matrix<double, 3, 1>::Constant(0);
		StateBuffer_[i].time_ = 0;
	}

	idx_state_ = 0;
	idx_P_ = 0;
	idx_time_ = 0;
	idx_meas_ = 0;

	StateBuffer_[idx_state_].p_ = p;
	StateBuffer_[idx_state_].v_ = v;
	StateBuffer_[idx_state_].q_ = q;
	StateBuffer_[idx_state_].b_w_ = b_w;
	StateBuffer_[idx_state_].b_a_ = b_a;
	StateBuffer_[idx_state_].L_ = L;
	StateBuffer_[idx_state_].q_wv_ = q_wv;
	StateBuffer_[idx_state_].q_ci_ = q_ci;
	StateBuffer_[idx_state_].p_ic_ = p_ic;
	StateBuffer_[idx_state_].q_mi_ = q_mi;
	StateBuffer_[idx_state_].p_ig_ = p_ig;
	StateBuffer_[idx_state_].p_vw_ = p_vw;
	StateBuffer_[idx_state_].alpha_ = alpha;
	StateBuffer_[idx_state_].beta_ = beta;
	StateBuffer_[idx_state_].w_m_ = w_m;
	StateBuffer_[idx_state_].q_int_ = StateBuffer_[idx_state_].q_wv_;
	StateBuffer_[idx_state_].a_m_ = a_m;
	StateBuffer_[idx_state_].time_ = ros::Time::now().toSec();

	if (P.maxCoeff()==0 && P.minCoeff()==0)
		StateBuffer_[idx_P_].P_ <<  0;
	else
		StateBuffer_[idx_P_].P_ = P;

	// constants
	g_ = g;

	// buffer for vision failure check
	qvw_inittimer_=1;
	qbuff_=Eigen::Matrix<double,nBuff_,4>::Constant(0);

	// increase state pointers
	idx_state_++;
	idx_P_++;

//	ROS_INFO_STREAM("init: \n"
//			<<"p: "<<StateBuffer_[(unsigned char)(idx_state_-1)].p_<<"\n"
//			<<"v: "<<StateBuffer_[(unsigned char)(idx_state_-1)].v_<<"\n"
//			<<"q: "<<StateBuffer_[(unsigned char)(idx_state_-1)].q_.coeffs()<<"\n"
//			<<"b_w: "<<StateBuffer_[(unsigned char)(idx_state_-1)].b_w_<<"\n"
//			<<"b_a: "<<StateBuffer_[(unsigned char)(idx_state_-1)].b_a_<<"\n"
//			<<"L: "<<StateBuffer_[(unsigned char)(idx_state_-1)].L_<<"\n"
//			<<"q_wv: "<<StateBuffer_[(unsigned char)(idx_state_-1)].q_wv_.coeffs()<<"\n"
//			<<"q_ci: "<<StateBuffer_[(unsigned char)(idx_state_-1)].q_ci_.coeffs()<<"\n"
//			<<"p_ic: "<<StateBuffer_[(unsigned char)(idx_state_-1)].p_ic_<<"\n"
//			<<"q_mi: "<<StateBuffer_[(unsigned char)(idx_state_-1)].q_mi_<<"\n"
//			<<"p_ig: "<<StateBuffer_[(unsigned char)(idx_state_-1)].p_ig_<<"\n"
//			<<"p_vw: "<<StateBuffer_[(unsigned char)(idx_state_-1)].p_vw_<<"\n"
//			<<"alpha: "<<StateBuffer_[(unsigned char)(idx_state_-1)].alpha_<<"\n"
//			<<"beta: "<<StateBuffer_[(unsigned char)(idx_state_-1)].beta_<<"\n"
//			<<"n_a: "<<n_a_<<"\n"
//			<<"n_ba_: "<<n_ba_<<"\n"
//			<<"n_w_: "<<n_w_<<"\n"
//			<<"n_bw: "<<n_bw_<<"\n"
//			<<"n_L_: "<<n_L_<<"\n"
//			<<"n_qwv_: "<<n_qwv_<<"\n"
//			<<"n_qci_: "<<n_qci_<<"\n"
//			<<"n_pic_: "<<n_pic_<<"\n"
//			<<"n_qmi_: "<<n_qmi_<<"\n"
//			<<"n_pig_: "<<n_pig_<<"\n"
//			<<"n_pvw_: "<<n_pvw_<<"\n"
//			<<"n_alpha_: "<<n_alpha_<<"\n"
//			<<"n_beta_: "<<n_beta_<<"\n"
//			);

	msgCorrect_.header.stamp = ros::Time::now();
	msgCorrect_.header.seq = 0;
	msgCorrect_.angular_velocity.x = 0;
	msgCorrect_.angular_velocity.y = 0;
	msgCorrect_.angular_velocity.z = 0;
	msgCorrect_.linear_acceleration.x = 0;
	msgCorrect_.linear_acceleration.y = 0;
	msgCorrect_.linear_acceleration.z = 0;
	msgCorrect_.state[0] = StateBuffer_[(unsigned char)(idx_state_-1)].p_[0];
	msgCorrect_.state[1] = StateBuffer_[(unsigned char)(idx_state_-1)].p_[1];
	msgCorrect_.state[2] = StateBuffer_[(unsigned char)(idx_state_-1)].p_[2];
	msgCorrect_.state[3] = StateBuffer_[(unsigned char)(idx_state_-1)].v_[0];
	msgCorrect_.state[4] = StateBuffer_[(unsigned char)(idx_state_-1)].v_[1];
	msgCorrect_.state[5] = StateBuffer_[(unsigned char)(idx_state_-1)].v_[2];
	msgCorrect_.state[6] = StateBuffer_[(unsigned char)(idx_state_-1)].q_.w();
	msgCorrect_.state[7] = StateBuffer_[(unsigned char)(idx_state_-1)].q_.x();
	msgCorrect_.state[8] = StateBuffer_[(unsigned char)(idx_state_-1)].q_.y();
	msgCorrect_.state[9] = StateBuffer_[(unsigned char)(idx_state_-1)].q_.z();
	msgCorrect_.state[10] = StateBuffer_[(unsigned char)(idx_state_-1)].b_w_[0];
	msgCorrect_.state[11] = StateBuffer_[(unsigned char)(idx_state_-1)].b_w_[1];
	msgCorrect_.state[12] = StateBuffer_[(unsigned char)(idx_state_-1)].b_w_[2];
	msgCorrect_.state[13] = StateBuffer_[(unsigned char)(idx_state_-1)].b_a_[0];
	msgCorrect_.state[14] = StateBuffer_[(unsigned char)(idx_state_-1)].b_a_[1];
	msgCorrect_.state[15] = StateBuffer_[(unsigned char)(idx_state_-1)].b_a_[2];
	msgCorrect_.flag=sensor_fusion_core::ext_ekf::initialization;
	pubCorrect_.publish(msgCorrect_);

	initialized_=true;

}


//void Pose_Filter::imuCallback(const sfly_msgs::ext_ekfConstPtr & msg){ // real
//void Pose_Filter::imuCallback(const sfly_msgs::ext_imuConstPtr & msg){ // debug, state prop here, old version
//void Pose_Filter::imuCallback(const sensor_fusion_core::ext_imuConstPtr & msg){ // debug, state prop here, old version
void Sensor_Fusion_Core::imuCallback(const sensor_msgs::ImuConstPtr & msg){ // debug, state prop here

	if(!initialized_)
		return;  ////////////// early abort!!

	StateBuffer_[idx_state_].time_ = msg->header.stamp.toSec();

	static int seq = 0;

	// get inputs
	StateBuffer_[idx_state_].a_m_ << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
	StateBuffer_[idx_state_].w_m_ << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
//	StateBuffer_[idx_state_].a_m_ << msg->acceleration.x, msg->acceleration.y, msg->acceleration.z;
//	StateBuffer_[idx_state_].w_m_ << msg->angularVelocity.x, msg->angularVelocity.y, msg->angularVelocity.z;
//	StateBuffer_[idx_state_].a_m_ << msg->acceleration.x, msg->acceleration.y, msg->acceleration.z;
//	StateBuffer_[idx_state_].w_m_ << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;

	//TODO: find out reason for acc spikes! (for now just eliminate them)
	static Eigen::Matrix<double,3,1> last_am = Eigen::Matrix<double,3,1>(0, 0, 0);
	if(StateBuffer_[idx_state_].a_m_.norm() > 50) StateBuffer_[idx_state_].a_m_ = last_am;
	else last_am = StateBuffer_[idx_state_].a_m_;

	if(!predictionMade_)
	{
		if (fabs(StateBuffer_[(unsigned char)(idx_state_)].time_-StateBuffer_[(unsigned char)(idx_state_-1)].time_)>5)
		{
			ROS_WARN_STREAM_THROTTLE(2, "large time-gap re-initializing to last state\n");
			StateBuffer_[(unsigned char)(idx_state_-1)].time_=StateBuffer_[(unsigned char)(idx_state_)].time_;
			return; // early abort!! (if timegap too big...
		}
	}
 // debug, state prop here
//	if(msg->flag==0)	// state propagation is made externally, so we read the actual state
//	{
//		StateBuffer_[idx_state_].p_ = Eigen::Matrix<double,3,1>(msg->state[0], msg->state[1], msg->state[2]);
//		StateBuffer_[idx_state_].v_ = Eigen::Matrix<double,3,1>(msg->state[3], msg->state[4], msg->state[5]);
//		StateBuffer_[idx_state_].q_ = Eigen::Quaternion<double>(msg->state[9], msg->state[6], msg->state[7], msg->state[8]);
//		StateBuffer_[idx_state_].b_w_ = Eigen::Matrix<double,3,1>(msg->state[10], msg->state[11], msg->state[12]);
//		StateBuffer_[idx_state_].b_a_ = Eigen::Matrix<double,3,1>(msg->state[13], msg->state[14], msg->state[15]);
//		StateBuffer_[idx_state_].L_ = double(msg->state[16]);
//		StateBuffer_[idx_state_].q_wv_ = Eigen::Quaternion<double>(msg->state[20], msg->state[19], msg->state[18],msg->state[19]);
//	}
//	else if(msg->flag!=0)	// otherwise let's do the state prop. here
		propagateState(StateBuffer_[idx_state_].time_-StateBuffer_[(unsigned char)(idx_state_-1)].time_);

	predictProcessCovariance(StateBuffer_[idx_P_].time_-StateBuffer_[(unsigned char)(idx_P_-1)].time_);

	predictionMade_ = true;

//	// publish propagated state
//	msgState_.header.stamp = ros::Time::now();
//	msgState_.header.seq = seq;
//	msgState_.data[0] = StateBuffer_[(unsigned char)(idx_state_-1)].p_[0];
//	msgState_.data[1] = StateBuffer_[(unsigned char)(idx_state_-1)].p_[1];
//	msgState_.data[2] = StateBuffer_[(unsigned char)(idx_state_-1)].p_[2];
//	msgState_.data[3] = StateBuffer_[(unsigned char)(idx_state_-1)].v_[0];
//	msgState_.data[4] = StateBuffer_[(unsigned char)(idx_state_-1)].v_[1];
//	msgState_.data[5] = StateBuffer_[(unsigned char)(idx_state_-1)].v_[2];
//	msgState_.data[6] = StateBuffer_[(unsigned char)(idx_state_-1)].q_.w();
//	msgState_.data[7] = StateBuffer_[(unsigned char)(idx_state_-1)].q_.x();
//	msgState_.data[8] = StateBuffer_[(unsigned char)(idx_state_-1)].q_.y();
//	msgState_.data[9] = StateBuffer_[(unsigned char)(idx_state_-1)].q_.z();
//	msgState_.data[10] = StateBuffer_[(unsigned char)(idx_state_-1)].b_w_[0];
//	msgState_.data[11] = StateBuffer_[(unsigned char)(idx_state_-1)].b_w_[1];
//	msgState_.data[12] = StateBuffer_[(unsigned char)(idx_state_-1)].b_w_[2];
//	msgState_.data[13] = StateBuffer_[(unsigned char)(idx_state_-1)].b_a_[0];
//	msgState_.data[14] = StateBuffer_[(unsigned char)(idx_state_-1)].b_a_[1];
//	msgState_.data[15] = StateBuffer_[(unsigned char)(idx_state_-1)].b_a_[2];
//	msgState_.data[16] = StateBuffer_[(unsigned char)(idx_state_-1)].L_;
//	msgState_.data[17] = StateBuffer_[(unsigned char)(idx_state_-1)].q_wv_.w();
//	msgState_.data[18] = StateBuffer_[(unsigned char)(idx_state_-1)].q_wv_.x();
//	msgState_.data[19] = StateBuffer_[(unsigned char)(idx_state_-1)].q_wv_.y();
//	msgState_.data[20] = StateBuffer_[(unsigned char)(idx_state_-1)].q_wv_.z();
//	msgState_.data[21] = StateBuffer_[(unsigned char)(idx_state_-1)].q_ci_.w();
//	msgState_.data[22] = StateBuffer_[(unsigned char)(idx_state_-1)].q_ci_.x();
//	msgState_.data[23] = StateBuffer_[(unsigned char)(idx_state_-1)].q_ci_.y();
//	msgState_.data[24] = StateBuffer_[(unsigned char)(idx_state_-1)].q_ci_.z();
//	msgState_.data[25] = StateBuffer_[(unsigned char)(idx_state_-1)].p_ic_[0];
//	msgState_.data[26] = StateBuffer_[(unsigned char)(idx_state_-1)].p_ic_[1];
//	msgState_.data[27] = StateBuffer_[(unsigned char)(idx_state_-1)].p_ic_[2];
//	pubState_.publish(msgState_);

//	ROS_INFO_STREAM("predict: \n"
//			<<"p: "<<StateBuffer_[(unsigned char)(idx_P_-1)].p_<<"\n"
//			<<"v: "<<StateBuffer_[(unsigned char)(idx_P_-1)].v_<<"\n"
//			<<"q: "<<StateBuffer_[(unsigned char)(idx_P_-1)].q_.coeffs()<<"\n"
//			<<"b_a: "<<StateBuffer_[(unsigned char)(idx_P_-1)].b_a_<<"\n"
//			<<"b_w: "<<StateBuffer_[(unsigned char)(idx_P_-1)].b_w_<<"\n"
//			<<"L_: "<<StateBuffer_[(unsigned char)(idx_P_-1)].L_<<"\n"
//			<<"q_wv: "<<StateBuffer_[(unsigned char)(idx_P_-1)].q_wv_.coeffs()<<"\n"
//			<<"q_ci: "<<StateBuffer_[(unsigned char)(idx_P_-1)].q_ci_.coeffs()<<"\n"
//			<<"p_ic: "<<StateBuffer_[(unsigned char)(idx_P_-1)].p_ic_<<"\n"
//			<<"q_mi: "<<StateBuffer_[(unsigned char)(idx_P_-1)].q_mi_<<"\n"
//			<<"p_ig: "<<StateBuffer_[(unsigned char)(idx_P_-1)].p_ig_<<"\n"
//			<<"p_vw: "<<StateBuffer_[(unsigned char)(idx_P_-1)].p_vw_<<"\n"
//			<<"alpha: "<<StateBuffer_[(unsigned char)(idx_P_-1)].alpha_<<"\n"
//			<<"beta: "<<StateBuffer_[(unsigned char)(idx_P_-1)].beta_<<"\n"
//			<<"n_a: "<<n_a_<<"\n"
//			<<"n_ba_: "<<n_ba_<<"\n"
//			<<"n_w_: "<<n_w_<<"\n"
//			<<"n_bw: "<<n_bw_<<"\n"
//			<<"n_L_: "<<n_L_<<"\n"
//			<<"n_qwv_: "<<n_qwv_<<"\n"
//			<<"n_qci_: "<<n_qci_<<"\n"
//			<<"n_pic_: "<<n_pic_<<"\n"
//			<<"n_qmi_: "<<n_qmi_<<"\n"
//			<<"n_pig_: "<<n_pig_<<"\n"
//			<<"n_pvw_: "<<n_pvw_<<"\n"
//			<<"n_alpha_: "<<n_alpha_<<"\n"
//			<<"n_beta_: "<<n_beta_<<"\n"
//			<<"dt state: "<<StateBuffer_[(unsigned char)(idx_state_-1)].time_-StateBuffer_[idx_state_-2].time_<<"\n"
//			<<"dt P: "<<StateBuffer_[(unsigned char)(idx_state_-1)].time_-StateBuffer_[idx_P_-2].time_<<"\n"
//			<<"a_m_: "<<StateBuffer_[(unsigned char)(idx_state_-1)].a_m_<<"\n"
//			<<"w_m_: "<<StateBuffer_[(unsigned char)(idx_state_-1)].w_m_<<"\n"
//			<<"idx_state_: "<<int(idx_state_)<<"\n"
//			<<"idx_state_ -1: "<<int((unsigned char)(idx_state_-1))<<"\n"
//			);

	msgPose_.header.stamp = ros::Time::now();
	msgPose_.header.seq = seq;
	geometry_msgs::Quaternion msgq;
	msgq.w=StateBuffer_[(unsigned char)(idx_state_-1)].q_.w(); msgq.x=StateBuffer_[(unsigned char)(idx_state_-1)].q_.x(); msgq.y=StateBuffer_[(unsigned char)(idx_state_-1)].q_.y(); msgq.z=StateBuffer_[(unsigned char)(idx_state_-1)].q_.z();
	msgPose_.pose.pose.orientation = msgq;
	geometry_msgs::Point msgp;
	msgp.x=StateBuffer_[(unsigned char)(idx_state_-1)].p_(0); msgp.y=StateBuffer_[(unsigned char)(idx_state_-1)].p_(1); msgp.z=StateBuffer_[(unsigned char)(idx_state_-1)].p_(2);
	msgPose_.pose.pose.position = msgp;
	boost::array<double, 36> cov;
//	if (filter_quality_ == BAD)
//		cov.assign(1e9);
//	else
	{	// put position and orientation covariance together...
		for (int i=0;i<9;i++)
		{
//			if(StateBuffer_[(unsigned char)(idx_state_-1)].P_(i/3*nState_ + i%3)>1)
//				ROS_WARN_STREAM("got a value above 1");
			cov[i/3*6 + i%3]=StateBuffer_[(unsigned char)(idx_state_-1)].P_(i/3*nState_ + i%3);
		}
		for (int i=0;i<9;i++)
		{
//			if(StateBuffer_[(unsigned char)(idx_state_-1)].P_(i/3*nState_ + (i%3+6))>1)
//				ROS_WARN_STREAM("got a value above 1");
			cov[i/3*6 + (i%3+3)]=StateBuffer_[(unsigned char)(idx_state_-1)].P_(i/3*nState_ + (i%3+6));
		}
		for (int i=0;i<9;i++)
		{
//			if(StateBuffer_[(unsigned char)(idx_state_-1)].P_((i/3+6)*nState_ + i%3)>1)
//				ROS_WARN_STREAM("got a value above 1");
			cov[(i/3+3)*6 + i%3]=StateBuffer_[(unsigned char)(idx_state_-1)].P_((i/3+6)*nState_ + i%3);
		}
		for (int i=0;i<9;i++)
		{
//			if(StateBuffer_[(unsigned char)(idx_state_-1)].P_((i/3+6)*nState_ + (i%3+6))>1)
//				ROS_WARN_STREAM("got a value above 1");
			cov[(i/3+3)*6 + (i%3+3)]=StateBuffer_[(unsigned char)(idx_state_-1)].P_((i/3+6)*nState_ + (i%3+6));
		}
	}
	msgPose_.pose.covariance = cov;
	pubPose_.publish(msgPose_);

	msgPoseCtrl_.header.stamp = ros::Time::now();
	msgPoseCtrl_.header.seq = seq;
	msgq.w=StateBuffer_[(unsigned char)(idx_state_-1)].q_.w(); msgq.x=StateBuffer_[(unsigned char)(idx_state_-1)].q_.x(); msgq.y=StateBuffer_[(unsigned char)(idx_state_-1)].q_.y(); msgq.z=StateBuffer_[(unsigned char)(idx_state_-1)].q_.z();
	msgPoseCtrl_.pose.orientation = msgq;
	msgp.x=StateBuffer_[(unsigned char)(idx_state_-1)].p_(0); msgp.y=StateBuffer_[(unsigned char)(idx_state_-1)].p_(1); msgp.z=StateBuffer_[(unsigned char)(idx_state_-1)].p_(2);
	msgPoseCtrl_.pose.position = msgp;
	geometry_msgs::Vector3 msgv;
	msgv.x=StateBuffer_[(unsigned char)(idx_state_-1)].v_(0); msgv.y=StateBuffer_[(unsigned char)(idx_state_-1)].v_(1); msgv.z=StateBuffer_[(unsigned char)(idx_state_-1)].v_(2);
	msgPoseCtrl_.velocity = msgv;

//	if (filter_quality_ == BAD)
//		cov.assign(1e9);
//	else
	if(sqrt(msgv.x*msgv.x+msgv.y*msgv.y+msgv.z*msgv.z)<1)
		pubPoseCrtl_.publish(msgPoseCtrl_);

	seq++;
}




void Sensor_Fusion_Core::stateCallback(const sensor_fusion_core::ext_ekfConstPtr & msg){

	if(!initialized_)
		return;  ////////////// early abort!!

	StateBuffer_[idx_state_].time_ = msg->header.stamp.toSec();

	static int seq = 0;

	// get inputs
	StateBuffer_[idx_state_].a_m_ << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
	StateBuffer_[idx_state_].w_m_ << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;

	//TODO: find out reason for acc spikes! (for now just eliminate them)
	static Eigen::Matrix<double,3,1> last_am = Eigen::Matrix<double,3,1>(0, 0, 0);
	if(StateBuffer_[idx_state_].a_m_.norm() > 50) StateBuffer_[idx_state_].a_m_ = last_am;
	else last_am = StateBuffer_[idx_state_].a_m_;

	bool isnumeric = checkForNumeric(&msg->state[0],10, "prediction p,v,q");


	if(!predictionMade_)
	{
		if (fabs(StateBuffer_[(unsigned char)(idx_state_)].time_-StateBuffer_[(unsigned char)(idx_state_-1)].time_)>5)
		{
			ROS_WARN_STREAM_THROTTLE(2, "large time-gap re-initializing to last state\n");
			StateBuffer_[(unsigned char)(idx_state_-1)].time_=StateBuffer_[(unsigned char)(idx_state_)].time_;
			return; // early abort!! (if timegap too big...
		}
	}

	if(msg->flag==sensor_fusion_core::ext_ekf::current_state && isnumeric)	// state propagation is made externally, so we read the actual state
	{
		StateBuffer_[idx_state_].p_ = Eigen::Matrix<double,3,1>(msg->state[0], msg->state[1], msg->state[2]);
		StateBuffer_[idx_state_].v_ = Eigen::Matrix<double,3,1>(msg->state[3], msg->state[4], msg->state[5]);
		StateBuffer_[idx_state_].q_ = Eigen::Quaternion<double>(msg->state[6], msg->state[7], msg->state[8], msg->state[9]);

		// zero props:
		StateBuffer_[idx_state_].b_w_ = StateBuffer_[(unsigned char)(idx_state_-1)].b_w_;
		StateBuffer_[idx_state_].b_a_ = StateBuffer_[(unsigned char)(idx_state_-1)].b_a_;
		StateBuffer_[idx_state_].L_ = StateBuffer_[(unsigned char)(idx_state_-1)].L_;
		StateBuffer_[idx_state_].q_wv_ = StateBuffer_[(unsigned char)(idx_state_-1)].q_wv_;
		StateBuffer_[idx_state_].q_ci_ = StateBuffer_[(unsigned char)(idx_state_-1)].q_ci_;
		StateBuffer_[idx_state_].p_ic_ = StateBuffer_[(unsigned char)(idx_state_-1)].p_ic_;
		idx_state_++;

		hl_state_buf_ = *msg;
	}
	else if(msg->flag==sensor_fusion_core::ext_ekf::ignore_state || !isnumeric)	// otherwise let's do the state prop. here
		propagateState(StateBuffer_[idx_state_].time_-StateBuffer_[(unsigned char)(idx_state_-1)].time_);


	predictProcessCovariance(StateBuffer_[idx_P_].time_-StateBuffer_[(unsigned char)(idx_P_-1)].time_);

	predictionMade_ = true;

	msgPoseCtrl_.header.stamp = ros::Time::now();
	msgPoseCtrl_.header.seq = seq;
	geometry_msgs::Quaternion msgq;
	msgq.w=StateBuffer_[(unsigned char)(idx_state_-1)].q_.w(); msgq.x=StateBuffer_[(unsigned char)(idx_state_-1)].q_.x(); msgq.y=StateBuffer_[(unsigned char)(idx_state_-1)].q_.y(); msgq.z=StateBuffer_[(unsigned char)(idx_state_-1)].q_.z();
	msgPoseCtrl_.pose.orientation = msgq;
	geometry_msgs::Point msgp;
	msgp.x=StateBuffer_[(unsigned char)(idx_state_-1)].p_(0); msgp.y=StateBuffer_[(unsigned char)(idx_state_-1)].p_(1); msgp.z=StateBuffer_[(unsigned char)(idx_state_-1)].p_(2);
	msgPoseCtrl_.pose.position = msgp;
	geometry_msgs::Vector3 msgv;
	msgv.x=StateBuffer_[(unsigned char)(idx_state_-1)].v_(0); msgv.y=StateBuffer_[(unsigned char)(idx_state_-1)].v_(1); msgv.z=StateBuffer_[(unsigned char)(idx_state_-1)].v_(2);
	msgPoseCtrl_.velocity = msgv;

	if(sqrt(msgv.x*msgv.x+msgv.y*msgv.y+msgv.z*msgv.z)<1)
		pubPoseCrtl_.publish(msgPoseCtrl_);

	seq++;
}




void Sensor_Fusion_Core::propagateState(const double dt )
{
	// zero props:
	StateBuffer_[idx_state_].b_w_ = StateBuffer_[(unsigned char)(idx_state_-1)].b_w_;
	StateBuffer_[idx_state_].b_a_ = StateBuffer_[(unsigned char)(idx_state_-1)].b_a_;
	StateBuffer_[idx_state_].L_ = StateBuffer_[(unsigned char)(idx_state_-1)].L_;
	StateBuffer_[idx_state_].q_wv_ = StateBuffer_[(unsigned char)(idx_state_-1)].q_wv_;
	StateBuffer_[idx_state_].q_ci_ = StateBuffer_[(unsigned char)(idx_state_-1)].q_ci_;
	StateBuffer_[idx_state_].p_ic_ = StateBuffer_[(unsigned char)(idx_state_-1)].p_ic_;
	StateBuffer_[idx_state_].q_mi_ = StateBuffer_[(unsigned char)(idx_state_-1)].q_mi_;
	StateBuffer_[idx_state_].p_ig_ = StateBuffer_[(unsigned char)(idx_state_-1)].p_ig_;
	StateBuffer_[idx_state_].p_vw_ = StateBuffer_[(unsigned char)(idx_state_-1)].p_vw_;
	StateBuffer_[idx_state_].alpha_ = StateBuffer_[(unsigned char)(idx_state_-1)].alpha_;
	StateBuffer_[idx_state_].beta_ = StateBuffer_[(unsigned char)(idx_state_-1)].beta_;

	Eigen::Quaternion<double> dq;
	Eigen::Matrix<double,3,1> dv;
	Eigen::Matrix<double,4,4> Omega;
	Eigen::Matrix<double,3,1> ew = StateBuffer_[idx_state_].w_m_ - StateBuffer_[idx_state_].b_w_;
	Eigen::Matrix<double,3,1> ewold = StateBuffer_[(unsigned char)(idx_state_-1)].w_m_ - StateBuffer_[(unsigned char)(idx_state_-1)].b_w_;
	Eigen::Matrix<double,3,1> ea = StateBuffer_[idx_state_].a_m_ - StateBuffer_[idx_state_].b_a_;
	Eigen::Matrix<double,3,1> eaold = StateBuffer_[(unsigned char)(idx_state_-1)].a_m_ - StateBuffer_[(unsigned char)(idx_state_-1)].b_a_;
	Omega <<  0,     ew[2], -ew[1], ew[0]
			 ,-ew[2],  0,      ew[0], ew[1]
			 ,ew[1], -ew[0],  0    ,  ew[2]
			 ,-ew[0], -ew[1], -ew[2],  0;
	Eigen::Matrix<double,4,4> OmegaOld;
	OmegaOld <<  0,     ewold[2], -ewold[1], ewold[0]
			 ,-ewold[2],  0,      ewold[0], ewold[1]
			 ,ewold[1], -ewold[0],  0    ,  ewold[2]
			 ,-ewold[0], -ewold[1], -ewold[2],  0;
	Eigen::Matrix<double,4,4> OmegaMean;
	OmegaMean <<  0,     (ew[2]+ewold[2])/2, -(ew[1]+ewold[1])/2, (ew[0]+ewold[0])/2
			 ,-(ew[2]+ewold[2])/2,  0,      (ew[0]+ewold[0])/2, (ew[1]+ewold[1])/2
			 ,(ew[1]+ewold[1])/2, -(ew[0]+ewold[0])/2,  0    ,  (ew[2]+ewold[2])/2
			 ,-(ew[0]+ewold[0])/2, -(ew[1]+ewold[1])/2, -(ew[2]+ewold[2])/2,  0;

//	StateBuffer_[idx_state_].q_ = (Eigen::Matrix<double,4,4>::Identity() + 0.5*Omega*dt)*StateBuffer_[(unsigned char)(idx_state_-1)].q_.coeffs();	//zero order quat integr.
	int div;
	div = 1;
	Eigen::Matrix<double,4,4> MatExp;
	MatExp = Eigen::Matrix<double,4,4>::Identity();
	OmegaMean *= 0.5*dt;
	for (int i = 1; i<5; i++)
	{
		div *= i;
		MatExp = MatExp + OmegaMean/div;
		OmegaMean *= OmegaMean;
	}
	StateBuffer_[idx_state_].q_.coeffs() = (MatExp + 1/48*(Omega*OmegaOld-OmegaOld*Omega)*dt*dt)*StateBuffer_[(unsigned char)(idx_state_-1)].q_.coeffs();		// first oder quat integr.
	StateBuffer_[idx_state_].q_int_.coeffs() = (MatExp + 1/48*(Omega*OmegaOld-OmegaOld*Omega)*dt*dt)*StateBuffer_[(unsigned char)(idx_state_-1)].q_int_.coeffs();		// first oder quat integr.
	StateBuffer_[idx_state_].q_int_.normalize();

	dv = (StateBuffer_[idx_state_].q_.toRotationMatrix()*ea+StateBuffer_[(unsigned char)(idx_state_-1)].q_.toRotationMatrix()*eaold)/2;
	StateBuffer_[idx_state_].v_ = StateBuffer_[(unsigned char)(idx_state_-1)].v_ + (dv - g_)*dt;
	StateBuffer_[idx_state_].p_ = StateBuffer_[(unsigned char)(idx_state_-1)].p_ + ((StateBuffer_[idx_state_].v_+StateBuffer_[(unsigned char)(idx_state_-1)].v_)/2*dt);


//	ROS_INFO_STREAM("predict state: idx = " <<  int(idx_state_) << "\n"
//			"        v= " <<  StateBuffer_[idx_state_].v_<< "\n"
//			"        eaold= " <<  eaold << "\n"
//			"        dv= " <<  dv << "\n"
//			"        dt = " <<  dt << "\n");

	idx_state_++;

}

void Sensor_Fusion_Core::predictProcessCovariance(const double dt){

	// scale noise with dt
	double na = n_a_;// / sqrt(dt);
	Eigen::Vector3d nav = Eigen::Vector3d(na,na,na);
	double nba= n_ba_;// * sqrt(dt);
	Eigen::Vector3d nbav = Eigen::Vector3d(nba,nba,nba);
	double nw = n_w_;// / sqrt(dt);
	Eigen::Vector3d nwv = Eigen::Vector3d(nw,nw,nw);
	double nbw= n_bw_;// * sqrt(dt);
	Eigen::Vector3d nbwv = Eigen::Vector3d(nbw,nbw,nbw);
	Eigen::Vector3d nqwvv = Eigen::Vector3d (n_qwv_,n_qwv_,n_qwv_);
	Eigen::Vector3d nqciv = Eigen::Vector3d (n_qci_,n_qci_,n_qci_);
	Eigen::Vector3d npicv = Eigen::Vector3d (n_pic_,n_pic_,n_pic_);
	Eigen::Vector3d nqmiv = Eigen::Vector3d (n_qmi_,n_qmi_,n_qmi_);
	Eigen::Vector3d npigv = Eigen::Vector3d (n_pig_,n_pig_,n_pig_);
	Eigen::Vector3d npvwv = Eigen::Vector3d (n_pvw_,n_pvw_,n_pvw_);
	double nalpha = n_alpha_;
	double nbeta = n_beta_;

	Eigen::Matrix<double,3,1> ew = StateBuffer_[idx_P_].w_m_ - StateBuffer_[idx_P_].b_w_;
	Eigen::Matrix<double,3,1> ea = StateBuffer_[idx_P_].a_m_ - StateBuffer_[idx_P_].b_a_;

	Eigen::Matrix<double,3,3> a_sk;
	a_sk << 0, -ea(2), ea(1)
			,ea(2), 0, -ea(0)
			,-ea(1), ea(0), 0;
	Eigen::Matrix<double,3,3> w_sk;
	w_sk << 0, -ew(2), ew(1)
			,ew(2), 0, -ew(0)
			,-ew(1), ew(0), 0;
	Eigen::Matrix<double,3,3> eye3 = Eigen::Matrix<double,3,3>::Identity();

	Eigen::Matrix<double,3,3> C_eq = StateBuffer_[idx_P_].q_.toRotationMatrix();

	Eigen::Matrix<double,3,3> Ca3 = C_eq*a_sk;
	Eigen::Matrix<double,3,3> A = Ca3*(-dt*dt/2*eye3 + dt*dt*dt/6*w_sk - dt*dt*dt*dt/24*w_sk*w_sk);
	Eigen::Matrix<double,3,3> B = Ca3*(dt*dt*dt/6*eye3 - dt*dt*dt*dt/24*w_sk + dt*dt*dt*dt*dt/120*w_sk*w_sk);
	Eigen::Matrix<double,3,3> D = -A;
	Eigen::Matrix<double,3,3> E = eye3 - dt*w_sk + dt*dt/2*w_sk*w_sk;
	Eigen::Matrix<double,3,3> F = -dt*eye3 + dt*dt/2*w_sk - dt*dt*dt/6*(w_sk*w_sk);
	Eigen::Matrix<double,3,3> C = Ca3*F;

	// discrete error state propagation Matrix Fd
	Fd_ = Eigen::Matrix<double,nState_,nState_>::Identity();
	Fd_.block(0,3,3,3) = dt*eye3;
	Fd_.block(0,6,3,3) = A;
	Fd_.block(0,9,3,3) = B;
	Fd_.block(0,12,3,3) = -C_eq*dt*dt/2;

	Fd_.block(3,6,3,3) = C;
	Fd_.block(3,9,3,3) = D;
	Fd_.block(3,12,3,3) = -C_eq*dt;

	Fd_.block(6,6,3,3) = E;
	Fd_.block(6,9,3,3) = F;

	calc_Q(dt, StateBuffer_[idx_P_].q_, ew, ea, nav, nbav, nwv, nbwv, n_L_, nqwvv, nqciv, npicv,
			nqmiv, npigv, npvwv, nalpha, nbeta, &Qd_);

	StateBuffer_[idx_P_].P_ = Fd_ *  StateBuffer_[(unsigned char)(idx_P_-1)].P_ * Fd_.transpose() + Qd_;

//	for(int i=0;i<nState_;++i)
//		for(int j=0;j<nState_;++j)
//			if(StateBuffer_[idx_P_].P_(i,j)<1e-9)
//			{
////				ROS_ERROR_STREAM("made nasty P correction at " << i << "/"<< j);
//				StateBuffer_[idx_P_].P_(i,j)=0;
//			}
//	StateBuffer_[idx_P_].P_(23,1)=1e-9;

//	ROS_INFO_STREAM("predict P: idx = " <<  int(idx_P_) << "\n"
//			"        dt = " << dt << "\n");


//	ROS_INFO_STREAM ("predict: \n"
//			<<"P_-1: \n"<<StateBuffer_[(unsigned char)(idx_P_-1)].P_<<"\n\n"
//			<<"P_: \n"<<StateBuffer_[idx_P_].P_<<"\n\n"
//			<<"Fd_.: \n"<<Fd_<<"\n\n"
//			<<"Qd_: \n"<<Qd_<<"\n\n");

	idx_P_++;
}
bool Sensor_Fusion_Core::getStateAtIdx(State* timestate, unsigned char idx)
{
	if(!predictionMade_)
	{
		timestate->time_=-1;
		return false;
	}

	timestate->p_ = StateBuffer_[idx].p_;
	timestate->v_ = StateBuffer_[idx].v_;
	timestate->q_ = StateBuffer_[idx].q_;
	timestate->b_w_ = StateBuffer_[idx].b_w_;
	timestate->b_a_ = StateBuffer_[idx].b_a_;
	timestate->L_ = StateBuffer_[idx].L_;
	timestate->q_wv_ =StateBuffer_[idx].q_wv_;
	timestate->q_ci_ =StateBuffer_[idx].q_ci_;
	timestate->p_ic_ =StateBuffer_[idx].p_ic_;
	timestate->q_mi_ =StateBuffer_[idx].q_mi_;
	timestate->p_ig_ =StateBuffer_[idx].p_ig_;
	timestate->p_vw_ =StateBuffer_[idx].p_vw_;
	timestate->alpha_ =StateBuffer_[idx].alpha_;
	timestate->beta_ =StateBuffer_[idx].beta_;
	timestate->P_ = StateBuffer_[idx].P_;
	timestate->time_ = StateBuffer_[idx].time_;
	timestate->w_m_ = StateBuffer_[idx].w_m_;
	timestate->q_int_ = StateBuffer_[idx].q_int_;
	timestate->a_m_ = StateBuffer_[idx].a_m_;

	return true;
}

unsigned char Sensor_Fusion_Core::getClosestState(State* timestate, ros::Time tstamp, double delay)
{
//	boost::mutex::scoped_lock lock(ekf_mutex_);
	if(!predictionMade_)
	{
		timestate->time_=-1;
		return false;
	}

	unsigned char idx = (unsigned char)(idx_state_-1);
	double timedist = 1e100;
	double timenow = tstamp.toSec()-delay-DELAY_;

	while (fabs(timenow-StateBuffer_[idx].time_)<timedist) // timedist decreases continuously until best point reached... then rises again
	{
		timedist = fabs(timenow-StateBuffer_[idx].time_);
		idx--;
	}
	idx++;								// we subtracted one too much before....

	static bool started=false;
	if (idx==1 && !started) idx=2;
	started=true;

	propPToIdx(idx);

//	ROS_INFO_STREAM("time index: " << int(idx) << "\n"
//			"timenow - idx_time_(idx): " << timenow-StateBuffer_[idx].time_ << "\n");
	timestate->p_ = StateBuffer_[idx].p_;
	timestate->v_ = StateBuffer_[idx].v_;
	timestate->q_ = StateBuffer_[idx].q_;
	timestate->b_w_ = StateBuffer_[idx].b_w_;
	timestate->b_a_ = StateBuffer_[idx].b_a_;
	timestate->L_ = StateBuffer_[idx].L_;
	timestate->q_wv_ =StateBuffer_[idx].q_wv_;
	timestate->q_ci_ =StateBuffer_[idx].q_ci_;
	timestate->p_ic_ =StateBuffer_[idx].p_ic_;
	timestate->q_mi_ =StateBuffer_[idx].q_mi_;
	timestate->p_ig_ =StateBuffer_[idx].p_ig_;
	timestate->p_vw_ =StateBuffer_[idx].p_vw_;
	timestate->alpha_ =StateBuffer_[idx].alpha_;
	timestate->beta_ =StateBuffer_[idx].beta_;
	timestate->P_ = StateBuffer_[idx].P_;
	timestate->time_ = StateBuffer_[idx].time_;
	timestate->w_m_ = StateBuffer_[idx].w_m_;
	timestate->q_int_ = StateBuffer_[idx].q_int_;
	timestate->a_m_ = StateBuffer_[idx].a_m_;

	return idx;
}


void Sensor_Fusion_Core::propPToIdx(unsigned char idx)
{
  // propagate cov matrix until idx
  if (idx<idx_state_ && (idx_P_<=idx || idx_P_>idx_state_))	//need to propagate some covs
	while (idx!=(unsigned char)(idx_P_-1))
		predictProcessCovariance(StateBuffer_[idx_P_].time_-StateBuffer_[(unsigned char)(idx_P_-1)].time_);
}

//bool Sensor_Fusion_Core::registerMeasurement(void (*measfunc)(char*, unsigned char), char* message, unsigned int msize, unsigned char stateidx,ros::Time tstamp)
//{
//	// insert measurement at correct buffer position (time sorted, newest at the beginning)
//	Meas m;
//	m.measfunc_ = measfunc;
//	m.message_ = (char*)malloc(msize);
//	memcpy(m.message_, message, msize);
//	m.msize_=msize;
//
//	double timedist = 1e100;
//	double timemeas = tstamp.toSec();
//
//	std::list<Meas>::iterator it=MeasBuffer_.begin();
//	while((it->time_-timemeas)<timedist && (it->time_-timemeas)>0 && it!=MeasBuffer_.end()) // timedist decreases continuously until best point reached... then rises again
//	{
//		timedist = it->time_-timemeas;
//		it++;
//	}
//	if(it!=MeasBuffer_.begin() && it!=MeasBuffer_.end())
//		it--;	// we substracted one too much in the while loop...
//	else if(it==MeasBuffer_.end())
//		return false;	///// EARLY ABORT! measurement too old to apply...
//
//	MeasBuffer_.insert(it,m);
//	if(MeasBuffer_.size()>nStateBuffer_)		// keep constant size of meas buffer
//	{
//		delete MeasBuffer_.end()->message_;
//		MeasBuffer_.erase(MeasBuffer_.end());
//	}
//
//	// crawl through the buffer from the insertion point till the beginning (i.e. apply all meas)
//	unsigned char idx_now = idx_state_; //save this so we know what time it is now
//	while(it!=MeasBuffer_.begin())
//	{
//		std::list<Meas>::iterator previt=it;
//		previt--;
//		idx_state_= previt->stateidx_+1;		// emulate fake now time until next measurement
//		it->measfunc_(it->message_, it->stateidx_);
//		it--;
//	}
//	idx_state_= idx_now;		// we arrived in the present now... and it should point to MeasBuffer_.begin()
//	it->measfunc_(it->message_, it->stateidx_);
//
//	return true;
//}


bool Sensor_Fusion_Core::applyMeasurement(unsigned char idx_delaystate, const MatrixXSd& H_delayed, const Eigen::VectorXd& res_delayed, const Eigen::MatrixXd& R_delayed,double fuzzythres){

//	boost::mutex::scoped_lock lock(ekf_mutex_);

	// get measurements
	if(!predictionMade_)
		return false;

	static int seq_m = 0;

//	ROS_INFO_STREAM("state to update: \n"
//			<<"p: "<<StateBuffer_[idx_delaystate].p_<<"\n"
//			<<"v: "<<StateBuffer_[idx_delaystate].v_<<"\n"
//			<<"q: "<<StateBuffer_[idx_delaystate].q_.coeffs()<<"\n"
//			<<"b_a: "<<StateBuffer_[idx_delaystate].b_a_<<"\n"
//			<<"b_w: "<<StateBuffer_[idx_delaystate].b_w_<<"\n"
//			<<"L_: "<<StateBuffer_[idx_delaystate].L_<<"\n"
//			<<"q_wv: "<<StateBuffer_[idx_delaystate].q_wv_.coeffs()<<"\n"
//			<<"q_ci: "<<StateBuffer_[idx_delaystate].q_ci_.coeffs()<<"\n"
//			<<"p_ic: "<<StateBuffer_[idx_delaystate].p_ic_<<"\n"
//			<<"q_mi: "<<StateBuffer_[idx_delaystate].q_mi_<<"\n"
//			<<"p_ig: "<<StateBuffer_[idx_delaystate].p_ig_<<"\n"
//			<<"p_vw: "<<StateBuffer_[idx_delaystate].p_vw_<<"\n"
//			<<"alpha: "<<StateBuffer_[idx_delaystate].alpha_<<"\n"
//			<<"beta: "<<StateBuffer_[idx_delaystate].beta_<<"\n"
//			<<"n_a: "<<n_a_<<"\n"
//			<<"n_ba_: "<<n_ba_<<"\n"
//			<<"n_w_: "<<n_w_<<"\n"
//			<<"n_bw: "<<n_bw_<<"\n"
//			<<"n_L_: "<<n_L_<<"\n"
//			<<"n_qwv_: "<<n_qwv_<<"\n"
//	);


	// check if all dimensions are correct, take H_delayed as "leader"
	unsigned char meas = H_delayed.rows();
	if(res_delayed.rows()!=meas || R_delayed.rows()!=meas || R_delayed.cols()!=meas)
	{
		ROS_WARN_STREAM("sizes of update matrices H and R do not match\n");
		return 0;	//////////////////// ***********early exit***************** /////////////////////
	}

	// make sure we have correctly propagated cov until idx_delaystate
	propPToIdx(idx_delaystate);

	S_.resize(meas,meas);
	K_.resize(nState_,meas);

	S_ = H_delayed * StateBuffer_[idx_delaystate].P_ * H_delayed.transpose() + R_delayed ;
	K_ = StateBuffer_[idx_delaystate].P_ * H_delayed.transpose() * S_.inverse();

//	ROS_INFO_STREAM("update inputs: \n"
//			<<"r: "<<res_delayed<<"\n"
//			<<"Hold: "<<H_delayed<<"\n"
//			<<"R: "<<R_delayed<<"\n"
//			<<"P: "<<StateBuffer_[idx_delaystate].P_<<"\n"
//			<<"K: "<<K_<<"\n"
//			<<"S: "<<S_<<"\n"
//			);

	correction_ = K_ * res_delayed;
	Eigen::Matrix<double,nState_, nState_> KH = (Eigen::Matrix<double,nState_,nState_>::Identity() - K_ * H_delayed);
	StateBuffer_[idx_delaystate].P_ =  KH * StateBuffer_[idx_delaystate].P_ * KH.transpose() + K_ * R_delayed * K_.transpose();

//	ROS_INFO_STREAM("correction: " << correction_ << "P: " << StateBuffer_[idx_delaystate].P_);

	if(fixedScale_){
		correction_(15) = 0; //scale
	}

	if(fixedBias_){
		correction_(9) = 0;  //acc bias x
		correction_(10) = 0; //acc bias y
		correction_(11) = 0; //acc bias z
		correction_(12) = 0; //gyro bias x
		correction_(13) = 0; //gyro bias y
		correction_(14) = 0; //gyro bias z
	}

	// state update:

	// store old values in case of FUZZY tracking
	// TODO: what to do with attitude? augment measurement noise?
	Eigen::Quaternion<double> buff_q = StateBuffer_[idx_delaystate].q_;
	Eigen::Matrix<double,3,1> buff_bw = StateBuffer_[idx_delaystate].b_w_;
	Eigen::Matrix<double,3,1> buff_ba = StateBuffer_[idx_delaystate].b_a_;
	double buff_L = StateBuffer_[idx_delaystate].L_;
	Eigen::Quaternion<double> buff_qwv = StateBuffer_[idx_delaystate].q_wv_;
	Eigen::Quaternion<double> buff_qci = StateBuffer_[idx_delaystate].q_ci_;
	Eigen::Matrix<double,3,1> buff_pic = StateBuffer_[idx_delaystate].p_ic_;
	Eigen::Quaternion<double> buff_qmi = StateBuffer_[idx_delaystate].q_mi_;
	Eigen::Matrix<double,3,1> buff_pig = StateBuffer_[idx_delaystate].p_ig_;
	Eigen::Matrix<double,3,1> buff_pvw = StateBuffer_[idx_delaystate].p_vw_;
	double buff_alpha = StateBuffer_[idx_delaystate].alpha_;
	double buff_beta = StateBuffer_[idx_delaystate].beta_;

	StateBuffer_[idx_delaystate].p_ = StateBuffer_[idx_delaystate].p_ + correction_.block(0,0,3,1);
	StateBuffer_[idx_delaystate].v_ = StateBuffer_[idx_delaystate].v_ + correction_.block(3,0,3,1);
	StateBuffer_[idx_delaystate].b_w_ = StateBuffer_[idx_delaystate].b_w_ + correction_.block(9,0,3,1);
	StateBuffer_[idx_delaystate].b_a_ = StateBuffer_[idx_delaystate].b_a_ + correction_.block(12,0,3,1);
	StateBuffer_[idx_delaystate].L_ = StateBuffer_[idx_delaystate].L_ + correction_(15);

	Eigen::Matrix<double,3,1> theta = correction_.block(6,0,3,1);
	Eigen::Quaternion<double> qbuff_q = Eigen::Quaternion<double>(1,0,0,0);
	if ((theta.transpose()*theta)(0)/4.0<1)
	{
		qbuff_q.w() = sqrt(1-(theta.transpose()*theta)(0)/4.0);
		qbuff_q.vec() = theta/2;
	}
	else
	{
		qbuff_q.w() = 1/sqrt(1+(theta.transpose()*theta)(0)/4.0);
		qbuff_q.vec() = (theta/2)/sqrt(1+(theta.transpose()*theta)(0)/4.0);
	}
	StateBuffer_[idx_delaystate].q_ = StateBuffer_[idx_delaystate].q_*qbuff_q;
	StateBuffer_[idx_delaystate].q_.normalize();

	theta = correction_.block(16,0,3,1);
	Eigen::Quaternion<double> qbuff_qwv = Eigen::Quaternion<double>(1,0,0,0);

        if ((theta.transpose()*theta)(0)/4.0<1)
        {
                qbuff_qwv.w() = sqrt(1-(theta.transpose()*theta)(0)/4.0);
                qbuff_qwv.vec() = theta/2;
        }
        else
        {
                qbuff_qwv.w() = 1/sqrt(1+(theta.transpose()*theta)(0)/4.0);
                qbuff_qwv.vec() = (theta/2)/sqrt(1+(theta.transpose()*theta)(0)/4.0);
        }
	StateBuffer_[idx_delaystate].q_wv_ = StateBuffer_[idx_delaystate].q_wv_*qbuff_qwv;
	StateBuffer_[idx_delaystate].q_wv_.normalize();

	theta = correction_.block(19,0,3,1);
	Eigen::Quaternion<double> qbuff_qci = Eigen::Quaternion<double>(1,0,0,0);
	if ((theta.transpose()*theta)(0)/4.0<1)
	{
		qbuff_qci.w() = sqrt(1-(theta.transpose()*theta)(0)/4.0);
		qbuff_qci.vec() = theta/2;
	}
	else
	{
		qbuff_qci.w() = 1/sqrt(1+(theta.transpose()*theta)(0)/4.0);
		qbuff_qci.vec() = (theta/2)/sqrt(1+(theta.transpose()*theta)(0)/4.0);
	}
	StateBuffer_[idx_delaystate].q_ci_ = StateBuffer_[idx_delaystate].q_ci_*qbuff_qci;
	StateBuffer_[idx_delaystate].q_ci_.normalize();

	StateBuffer_[idx_delaystate].p_ic_ = StateBuffer_[idx_delaystate].p_ic_ + correction_.block(22,0,3,1);

	theta = correction_.block(25,0,3,1);
	Eigen::Quaternion<double> qbuff_qmi = Eigen::Quaternion<double>(1,0,0,0);
	if ((theta.transpose()*theta)(0)/4.0<1)
	{
		qbuff_qmi.w() = sqrt(1-(theta.transpose()*theta)(0)/4.0);
		qbuff_qmi.vec() = theta/2;
	}
	else
	{
		qbuff_qmi.w() = 1/sqrt(1+(theta.transpose()*theta)(0)/4.0);
		qbuff_qmi.vec() = (theta/2)/sqrt(1+(theta.transpose()*theta)(0)/4.0);
	}
	StateBuffer_[idx_delaystate].q_mi_ = StateBuffer_[idx_delaystate].q_mi_*qbuff_qmi;

	StateBuffer_[idx_delaystate].p_ig_ = StateBuffer_[idx_delaystate].p_ig_ + correction_.block(28,0,3,1);
	StateBuffer_[idx_delaystate].p_vw_ = StateBuffer_[idx_delaystate].p_vw_ + correction_.block(31,0,3,1);
	StateBuffer_[idx_delaystate].alpha_ = StateBuffer_[idx_delaystate].alpha_ + correction_(34,0);
	StateBuffer_[idx_delaystate].beta_ = StateBuffer_[idx_delaystate].beta_ + correction_(35,0);

	// update qbuff_ and check for fuzzy tracking
	if (qvw_inittimer_>nBuff_)
	{
		Eigen::Quaternion<double> errq = StateBuffer_[idx_delaystate].q_wv_.conjugate()*Eigen::Quaternion<double>(getMedian(qbuff_.block(0,3,nBuff_,1)), getMedian(qbuff_.block(0,0,nBuff_,1)),getMedian(qbuff_.block(0,1,nBuff_,1)),getMedian(qbuff_.block(0,2,nBuff_,1)));	// should be unit quaternion if no error
		if  (std::max(errq.vec().maxCoeff(), -errq.vec().minCoeff())/fabs(errq.w())*2>fuzzythres)	// fuzzy tracking (small angle approx)
		{
			ROS_WARN_STREAM("fuzzy tracking triggered: " << std::max(errq.vec().maxCoeff(), -errq.vec().minCoeff())/fabs(errq.w())*2 << "\n");

			//state_.q_ = buff_q;
			StateBuffer_[idx_delaystate].b_w_ = buff_bw;
			StateBuffer_[idx_delaystate].b_a_ = buff_ba;
			StateBuffer_[idx_delaystate].L_ = buff_L;
			StateBuffer_[idx_delaystate].q_wv_ = buff_qwv;
			StateBuffer_[idx_delaystate].q_ci_ = buff_qci;
			StateBuffer_[idx_delaystate].p_ic_ = buff_pic;
			StateBuffer_[idx_delaystate].q_mi_ = buff_qmi;
			StateBuffer_[idx_delaystate].p_ig_ = buff_pig;
			StateBuffer_[idx_delaystate].p_vw_ = buff_pvw;
			StateBuffer_[idx_delaystate].alpha_ = buff_alpha;
			StateBuffer_[idx_delaystate].beta_ = buff_beta;
			correction_.block(9,0,16,1)=Eigen::Matrix<double,16,1>::Zero();
			qbuff_q = Eigen::Quaternion<double>(1,0,0,0);
			qbuff_qwv = Eigen::Quaternion<double>(1,0,0,0);
			qbuff_qci = Eigen::Quaternion<double>(1,0,0,0);
			qbuff_qmi = Eigen::Quaternion<double>(1,0,0,0);
		}
		else			////////////////// if ok update mean and 3sigma of past N q_vw's //////////////////////////
		{
			qbuff_.block(qvw_inittimer_-nBuff_-1,0,1,4) = Eigen::Matrix<double,1,4>(StateBuffer_[idx_delaystate].q_wv_.coeffs());
			qvw_inittimer_ = (qvw_inittimer_)%nBuff_+nBuff_+1;
		}
	}
	else	////////////////// at beginning get mean and 3sigma of past N q_vw's ///////////////////////
	{
		qbuff_.block(qvw_inittimer_-1,0,1,4) = Eigen::Matrix<double,1,4>(StateBuffer_[idx_delaystate].q_wv_.coeffs());
		qvw_inittimer_++;
	}

	idx_time_ = idx_state_;
	idx_state_ = idx_delaystate+1;
	idx_P_ = idx_delaystate+1;
	// propagate state matrix until now
	while (idx_state_!=idx_time_)
	  propagateState(StateBuffer_[idx_state_].time_-StateBuffer_[(unsigned char)(idx_state_-1)].time_);


//	ROS_INFO_STREAM("s,k,corr: \n"
//			<<"S_: "<<S_<<"\n"
//			<<"K_ "<<K_<<"\n"
//			<<"correction_ "<<correction_<<"\n"
//			);




//	ROS_INFO_STREAM("update: \n"
//			<<"p: "<<StateBuffer_[idx_state_-1].p_<<"\n"
//			<<"v: "<<StateBuffer_[idx_state_-1].v_<<"\n"
//			<<"q: "<<StateBuffer_[idx_state_-1].q_.coeffs()<<"\n"
//			<<"b_a: "<<StateBuffer_[idx_state_-1].b_a_<<"\n"
//			<<"b_w: "<<StateBuffer_[idx_state_-1].b_w_<<"\n"
//			<<"L_: "<<StateBuffer_[idx_state_-1].L_<<"\n"
//			<<"q_wv: "<<StateBuffer_[idx_state_-1].q_wv_.coeffs()<<"\n"
//			<<"q_ci: "<<StateBuffer_[idx_state_-1].q_ci_.coeffs()<<"\n"
//			<<"p_ic: "<<StateBuffer_[idx_state_-1].p_ic_<<"\n"
//			<<"q_mi: "<<StateBuffer_[idx_state_-1].q_mi_<<"\n"
//			<<"p_ig: "<<StateBuffer_[idx_state_-1].p_ig_<<"\n"
//			<<"p_vw: "<<StateBuffer_[idx_state_-1].p_vw_<<"\n"
//			<<"alpha: "<<StateBuffer_[idx_state_-1].alpha_<<"\n"
//			<<"beta: "<<StateBuffer_[idx_state_-1].beta_<<"\n"
//	);

	checkForNumeric(&correction_[0], HLI_EKF_STATE_SIZE, "update");

	// publish correction
	msgCorrect_.header.stamp = ros::Time::now();
	msgCorrect_.header.seq = seq_m;
	msgCorrect_.angular_velocity.x = 0;
	msgCorrect_.angular_velocity.y = 0;
	msgCorrect_.angular_velocity.z = 0;
	msgCorrect_.linear_acceleration.x = 0;
	msgCorrect_.linear_acceleration.y = 0;
	msgCorrect_.linear_acceleration.z = 0;

	const unsigned char idx = (unsigned char)(idx_state_-1);
	msgCorrect_.state[0] = StateBuffer_[idx].p_[0] - hl_state_buf_.state[0];
	msgCorrect_.state[1] = StateBuffer_[idx].p_[1] - hl_state_buf_.state[1];
	msgCorrect_.state[2] = StateBuffer_[idx].p_[2] - hl_state_buf_.state[2];
	msgCorrect_.state[3] = StateBuffer_[idx].v_[0] - hl_state_buf_.state[3];
	msgCorrect_.state[4] = StateBuffer_[idx].v_[1] - hl_state_buf_.state[4];
	msgCorrect_.state[5] = StateBuffer_[idx].v_[2] - hl_state_buf_.state[5];

	Eigen::Quaterniond hl_q(hl_state_buf_.state[6], hl_state_buf_.state[7], hl_state_buf_.state[8], hl_state_buf_.state[9]);
	qbuff_q = hl_q.inverse()*StateBuffer_[idx].q_;
	msgCorrect_.state[6] = qbuff_q.w();
	msgCorrect_.state[7] = qbuff_q.x();
	msgCorrect_.state[8] = qbuff_q.y();
	msgCorrect_.state[9] = qbuff_q.z();

	msgCorrect_.state[10] = StateBuffer_[idx].b_w_[0] - hl_state_buf_.state[10];
	msgCorrect_.state[11] = StateBuffer_[idx].b_w_[1] - hl_state_buf_.state[11];
	msgCorrect_.state[12] = StateBuffer_[idx].b_w_[2] - hl_state_buf_.state[12];
	msgCorrect_.state[13] = StateBuffer_[idx].b_a_[0] - hl_state_buf_.state[13];
	msgCorrect_.state[14] = StateBuffer_[idx].b_a_[1] - hl_state_buf_.state[14];
	msgCorrect_.state[15] = StateBuffer_[idx].b_a_[2] - hl_state_buf_.state[15];

	msgCorrect_.flag=sensor_fusion_core::ext_ekf::state_correction;
	pubCorrect_.publish(msgCorrect_);

	msgState_.header = msgCorrect_.header;
	msgState_.data[0] = StateBuffer_[(unsigned char)(idx_state_-1)].p_[0];
	msgState_.data[1] = StateBuffer_[(unsigned char)(idx_state_-1)].p_[1];
	msgState_.data[2] = StateBuffer_[(unsigned char)(idx_state_-1)].p_[2];
	msgState_.data[3] = StateBuffer_[(unsigned char)(idx_state_-1)].v_[0];
	msgState_.data[4] = StateBuffer_[(unsigned char)(idx_state_-1)].v_[1];
	msgState_.data[5] = StateBuffer_[(unsigned char)(idx_state_-1)].v_[2];
	msgState_.data[6] = StateBuffer_[(unsigned char)(idx_state_-1)].q_.w();
	msgState_.data[7] = StateBuffer_[(unsigned char)(idx_state_-1)].q_.x();
	msgState_.data[8] = StateBuffer_[(unsigned char)(idx_state_-1)].q_.y();
	msgState_.data[9] = StateBuffer_[(unsigned char)(idx_state_-1)].q_.z();
	msgState_.data[10] = StateBuffer_[(unsigned char)(idx_state_-1)].b_w_[0];
	msgState_.data[11] = StateBuffer_[(unsigned char)(idx_state_-1)].b_w_[1];
	msgState_.data[12] = StateBuffer_[(unsigned char)(idx_state_-1)].b_w_[2];
	msgState_.data[13] = StateBuffer_[(unsigned char)(idx_state_-1)].b_a_[0];
	msgState_.data[14] = StateBuffer_[(unsigned char)(idx_state_-1)].b_a_[1];
	msgState_.data[15] = StateBuffer_[(unsigned char)(idx_state_-1)].b_a_[2];
	msgState_.data[16] = StateBuffer_[(unsigned char)(idx_state_-1)].L_;
	msgState_.data[17] = StateBuffer_[(unsigned char)(idx_state_-1)].q_wv_.w();
	msgState_.data[18] = StateBuffer_[(unsigned char)(idx_state_-1)].q_wv_.x();
	msgState_.data[19] = StateBuffer_[(unsigned char)(idx_state_-1)].q_wv_.y();
	msgState_.data[20] = StateBuffer_[(unsigned char)(idx_state_-1)].q_wv_.z();
	msgState_.data[21] = StateBuffer_[(unsigned char)(idx_state_-1)].q_ci_.w();
	msgState_.data[22] = StateBuffer_[(unsigned char)(idx_state_-1)].q_ci_.x();
	msgState_.data[23] = StateBuffer_[(unsigned char)(idx_state_-1)].q_ci_.y();
	msgState_.data[24] = StateBuffer_[(unsigned char)(idx_state_-1)].q_ci_.z();
	msgState_.data[25] = StateBuffer_[(unsigned char)(idx_state_-1)].p_ic_[0];
	msgState_.data[26] = StateBuffer_[(unsigned char)(idx_state_-1)].p_ic_[1];
	msgState_.data[27] = StateBuffer_[(unsigned char)(idx_state_-1)].p_ic_[2];
	msgState_.data[28] = StateBuffer_[(unsigned char)(idx_state_-1)].q_mi_.w();
	msgState_.data[29] = StateBuffer_[(unsigned char)(idx_state_-1)].q_mi_.x();
	msgState_.data[30] = StateBuffer_[(unsigned char)(idx_state_-1)].q_mi_.y();
	msgState_.data[31] = StateBuffer_[(unsigned char)(idx_state_-1)].q_mi_.z();
	msgState_.data[32] = StateBuffer_[(unsigned char)(idx_state_-1)].p_ig_[0];
	msgState_.data[33] = StateBuffer_[(unsigned char)(idx_state_-1)].p_ig_[1];
	msgState_.data[34] = StateBuffer_[(unsigned char)(idx_state_-1)].p_ig_[2];
	msgState_.data[35] = StateBuffer_[(unsigned char)(idx_state_-1)].p_vw_[0];
	msgState_.data[36] = StateBuffer_[(unsigned char)(idx_state_-1)].p_vw_[1];
	msgState_.data[37] = StateBuffer_[(unsigned char)(idx_state_-1)].p_vw_[2];
	msgState_.data[38] = StateBuffer_[(unsigned char)(idx_state_-1)].alpha_;
	msgState_.data[39] = StateBuffer_[(unsigned char)(idx_state_-1)].beta_;
	pubState_.publish(msgState_);
	seq_m++;

	return 1;
}

void Sensor_Fusion_Core::Config(sensor_fusion_core::Sensor_Fusion_CoreConfig& config, uint32_t level){
	for(std::vector<CallbackType>::iterator it=callbacks_.begin();it!=callbacks_.end();it++)
		(*it)(config,level);
}


void Sensor_Fusion_Core::DynConfig(sensor_fusion_core::Sensor_Fusion_CoreConfig& config, uint32_t level){
//	if(level & sensor_fusion_core::Sensor_Fusion_Core_MISC)
//	{
		this->setFixedScale(config.fixed_scale);
		this->setFixedBias(config.fixed_bias);
		this->setNoiseAcc(config.noise_acc);
		this->setNoiseAccBias(config.noise_accbias);
		this->setNoiseGyr(config.noise_gyr);
		this->setNoiseGyrBias(config.noise_gyrbias);
		this->setNoiseScale(config.noise_scale);
		this->setNoiseWV(config.noise_qwv);
		this->setNoiseQCI(config.noise_qci);
		this->setNoisePIC(config.noise_pic);
		this->setNoiseQMI(config.noise_qmi);
		this->setNoisePIG(config.noise_pig);
		this->setNoisePVW(config.noise_pvw);
		this->setNoiseALPHA(config.noise_alpha);
		this->setNoiseBETA(config.noise_beta);
		this->setDELAY(config.delay);
//	}
}
