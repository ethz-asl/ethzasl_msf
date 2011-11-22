/*
 * vision_pose.cpp
 *
 *  Created on: Sep 30, 2010
 *      Author: sweiss
 */

#include "vision_pose.h"
#include "vismaggps_measurements.h"

// measurement size
#define nMeas_ 6

void VisionPoseHandler::subscribe(){

	ros::NodeHandle nh("pose_filter");
	subMeasurement_ = nh.subscribe("vision_pose", 1, &VisionPoseHandler::measurementCallback, this);

	nh.param("/vismaggps_fusion/meas_noise1",n_zp_,0.01);
	nh.param("/vismaggps_fusion/meas_noise2",n_zq_,0.02);

	// setup: initial pos, att, of measurement sensor
	VisMagGPSMeasurements* customMeas = (VisMagGPSMeasurements*)(measurements);
	customMeas->poseFilter_.registerCallback(&VisionPoseHandler::noiseConfig, this);
	customMeas->p_vc_ = Eigen::Matrix<double, 3, 1>::Constant(0);
	customMeas->q_cv_ = Eigen::Quaternion<double>(1, 0, 0, 0);
}

void VisionPoseHandler::noiseConfig(sensor_fusion_core::Sensor_Fusion_CoreConfig& config, uint32_t level)
{
	//	if(level & pose_filter::Pose_Filter_MISC)
	//	{
		this->n_zp_ = config.meas_noise1;
		this->n_zq_ = config.meas_noise2;
	//	}
}



//void VisionPoseHandler::executeMeasurement(char* vmsg, unsigned char idx)
void VisionPoseHandler::measurementCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr & msg)
{
//	const geometry_msgs::PoseWithCovarianceStampedConstPtr & msg = (const geometry_msgs::PoseWithCovarianceStampedConstPtr & )vmsg;
//	ROS_INFO_STREAM("measurement received \n"
//					<< "type is: " << typeid(msg).name());

	State state_old;
	ros::Time time_old = msg->header.stamp;
	Sensor_Fusion_Core::MatrixXSd H_old = Eigen::Matrix<double,nMeas_,nState_>::Constant(0);
	Eigen::VectorXd r_old(nMeas_);


	z_p_ = Eigen::Matrix<double,3,1>(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
	z_q_ = Eigen::Quaternion<double>(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
	z_p_ = -z_q_.toRotationMatrix().transpose()*z_p_;

	VisMagGPSMeasurements* customMeas = (VisMagGPSMeasurements*)(measurements);
	customMeas->p_vc_ = z_p_;
	customMeas->q_cv_ = z_q_;


	Eigen::MatrixXd R = Eigen::Matrix<double,nMeas_,nMeas_>::Constant(0);
//	Eigen::Matrix<double,nMeas_,1> buffvec;
//	buffvec  << n_zp_,n_zp_,n_zp_,n_zq_,n_zq_,n_zq_,1e-6;
//	R = buffvec.asDiagonal();
	R.block(0,0,6,6) = Eigen::Matrix<double,6,6>(&msg->pose.covariance[0]);

//	ROS_INFO_STREAM("R " << R << "\n");

	Eigen::Matrix<double,nMeas_,1> buffvec = R.diagonal(); //Eigen::Matrix<double,6,1>::Constant(0.01);
	//TODO: fix cov matrix in ptam --> for now just take diagonal
//	ROS_INFO_STREAM("measurements: " << "\n"
//			<< "z_p: " << z_p_ << "\n"
//			<< "z_q: " << z_q_.coeffs() << "\n"
//			<< "R(1,1): " << R(0,0) << "\n");
	if (R(0,0)<1e3)
	{
		R = Eigen::Matrix<double,nMeas_,nMeas_>::Constant(0);
		R = buffvec.asDiagonal();

//		bool ret = measurements->poseFilter_.getStateAtIdx(&state_old,idx);
//		if (state_old.time_ == -1 || ret==false)
//			return;		/// no prediction made yet, EARLY ABORT
		unsigned char idx = measurements->poseFilter_.getClosestState(&state_old,time_old);
		if (state_old.time_ == -1)
			return;		/// no prediction made yet, EARLY ABORT

//		ROS_INFO_STREAM("meas state: \n"
//				<<"p: "<<state_old.p_<<"\n"
//				<<"v: "<<state_old.v_<<"\n"
//				<<"q: "<<state_old.q_.coeffs()<<"\n"
//				<<"b_a: "<<state_old.b_a_<<"\n"
//				<<"b_w: "<<state_old.b_w_<<"\n"
//				<<"L: "<<state_old.L_<<"\n"
//				<<"q_wv: "<<state_old.q_wv_.coeffs()<<"\n"
//				<<"p_ic: "<<p_ic_<<"\n"
//				);

		Eigen::Matrix<double,3,3> C_wv = state_old.q_wv_.conjugate().toRotationMatrix();
		Eigen::Matrix<double,3,3> C_q = state_old.q_.conjugate().toRotationMatrix();
		Eigen::Matrix<double,3,3> C_ci = state_old.q_ci_.conjugate().toRotationMatrix();


		Eigen::Matrix<double,3,3> skewold;
		Eigen::Matrix<double,3,1> vecold;
		vecold = (state_old.p_+C_q.transpose()*state_old.p_ic_)*state_old.L_;
		skewold << 0, -vecold(2), vecold(1)
				,vecold(2), 0, -vecold(0)
				,-vecold(1), vecold(0), 0;

		Eigen::Matrix<double,3,3> pic_sk;
		pic_sk << 0, -state_old.p_ic_(2), state_old.p_ic_(1)
				,state_old.p_ic_(2), 0, -state_old.p_ic_(0)
				,-state_old.p_ic_(1), state_old.p_ic_(0), 0;


		// construct H matrix using H-blockx :-)
		H_old.block(0,0,3,3) = C_wv.transpose()*state_old.L_;
		H_old.block(0,6,3,3) = -C_wv.transpose()*C_q.transpose()*pic_sk*state_old.L_;
		H_old.block(0,15,3,1) = C_wv.transpose()*C_q.transpose()*state_old.p_ic_ + C_wv.transpose()*state_old.p_+state_old.p_vw_;
		H_old.block(0,16,3,3) = -C_wv.transpose()*skewold;
		H_old.block(0,22,3,3) = C_wv.transpose()*C_q.transpose()*state_old.L_;
		H_old.block(0,31,3,3) = Eigen::Matrix<double,3,3>::Identity()*state_old.L_;
		H_old.block(3,6,3,3) = C_ci;
		H_old.block(3,16,3,3) = C_ci*C_q;
		H_old.block(3,19,3,3) = Eigen::Matrix<double,3,3>::Identity();

		Eigen::Quaternion<double> q_err;
		q_err = (state_old.q_wv_*state_old.q_*state_old.q_ci_).conjugate()*z_q_.conjugate();
		r_old.block(0,0,3,1) = z_p_ - (state_old.p_vw_+C_wv.transpose()*(state_old.p_ + C_q.transpose()*state_old.p_ic_))*state_old.L_;
		r_old.block(3,0,3,1) = q_err.vec()/q_err.w()*2;

		measurements->poseFilter_.applyMeasurement(idx,H_old,r_old,R);
	}
}


//void VisionPoseHandler::measurementCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr & msg)
//{
//	State state_old;	// this is not really needed, we are only interested in the index at this point
//	ros::Time time_old = msg->header.stamp;
//	geometry_msgs::PoseWithCovarianceStamped buffmsg = *msg;
//
//	unsigned char idx = measurements->poseFilter_.getClosestState(&state_old,time_old);
//	if (state_old.time_ == -1)
//		return;		/// no prediction made yet, EARLY ABORT
//	measurements->poseFilter_.registerMeasurement(&VisionPoseHandler::executeMeasurement, (char*)&buffmsg, sizeof(geometry_msgs::PoseWithCovarianceStamped), idx,time_old);
//}
