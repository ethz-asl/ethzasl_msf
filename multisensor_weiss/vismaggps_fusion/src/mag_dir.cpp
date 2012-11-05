/*
 * mag_dir.cpp
 *
 *  Created on: Sep 30, 2010
 *      Author: sweiss
 */

#include "mag_dir.h"
#include "vismaggps_measurements.h"

// measurement size
#define nMeas_ 3

void MagDirHandler::subscribe(){

	ros::NodeHandle nh("pose_filter");
	subMeasurement_ = nh.subscribe("mag_dir", 1, &MagDirHandler::measurementCallback, this);

	nh.param("/leica_filter/meas_noise1",n_zm_,0.0001);

	VisMagGPSMeasurements* customMeas = (VisMagGPSMeasurements*)(measurements);
	customMeas->poseFilter_.registerCallback(&MagDirHandler::noiseConfig, this);
	customMeas->p_m_ = Eigen::Matrix<double, 3, 1>::Constant(0);

}

void MagDirHandler::noiseConfig(sensor_fusion_core::Sensor_Fusion_CoreConfig& config, uint32_t level){
	//	if(level & sensor_fusion_core::Sensor_Fusion_Core_MISC)
	//	{
	this->n_zm_ = config.meas_noise1;
	//	}
}

void MagDirHandler::measurementCallback(const geometry_msgs::PointStampedConstPtr & msg)
{
//	ROS_INFO_STREAM("measurement received \n");

	State state_old;
	ros::Time time_old = msg->header.stamp;
	Sensor_Fusion_Core::MatrixXSd H_old = Eigen::Matrix<double,nMeas_,nState_>::Constant(0);
	Eigen::VectorXd r_old(nMeas_);

	z_m_ = Eigen::Matrix<double,3,1>(msg->point.x, msg->point.y, msg->point.z);

	Eigen::MatrixXd R = Eigen::Matrix<double,nMeas_,nMeas_>::Constant(0);
	Eigen::Matrix<double,nMeas_,1> buffvec;
	buffvec  << n_zm_,n_zm_,n_zm_;
	R = buffvec.asDiagonal();

	VisMagGPSMeasurements* customMeas = (VisMagGPSMeasurements*)(measurements);
	customMeas->p_m_ = z_m_;

//	ROS_INFO_STREAM("R " << R << "\n");

//	ROS_INFO_STREAM("measurements: " << "\n"
//			<< "z_p_: " << z_p_ << "\n");
	if (R(1,1)<1e3)
	{
		unsigned char idx = measurements->poseFilter_.getClosestState(&state_old,time_old,0);
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
		Eigen::Matrix<double,3,3> C_mi = state_old.q_mi_.conjugate().toRotationMatrix();

		Eigen::Matrix<double,3,1> vec_m;
		vec_m << sin(state_old.beta_)*cos(state_old.alpha_), cos(state_old.beta_)*cos(state_old.alpha_), sin(state_old.alpha_);
		Eigen::Matrix<double,3,1> vec_m_dalpha;
		vec_m_dalpha << -sin(state_old.beta_)*sin(state_old.alpha_), -cos(state_old.beta_)*sin(state_old.alpha_), cos(state_old.alpha_);
		Eigen::Matrix<double,3,1> vec_m_dbeta;
		vec_m_dbeta << cos(state_old.beta_)*cos(state_old.alpha_), -sin(state_old.beta_)*cos(state_old.alpha_), 0;

		Eigen::Matrix<double,3,3> mw_sk1;
		Eigen::Matrix<double,3,1> vec1_mw = C_q*vec_m;
		mw_sk1 << 0, -vec1_mw(2), vec1_mw(1)
				,vec1_mw(2), 0, -vec1_mw(0)
				,-vec1_mw(1), vec1_mw(0), 0;

		Eigen::Matrix<double,3,3> mw_sk2;
		Eigen::Matrix<double,3,1> vec2_mw = C_mi*C_q*vec_m;
		mw_sk2 << 0, -vec2_mw(2), vec2_mw(1)
				,vec2_mw(2), 0, -vec2_mw(0)
				,-vec2_mw(1), vec2_mw(0), 0;

		// construct H matrix using H-blockx :-)
		H_old.block(0,6,3,3) = C_mi*mw_sk1;
		H_old.block(0,25,3,3) = mw_sk2;
		H_old.block(0,34,3,1) = C_mi*C_q*vec_m_dalpha;
		H_old.block(0,35,3,1) =C_mi*C_q*vec_m_dbeta;

		r_old.block(0,0,3,1) = z_m_ - C_mi*C_q*(vec_m);

//		ROS_INFO_STREAM("measurements: " << "\n"
//				<< "L: " << state_old.L_ << "\n"
//				<< "rmeas: " << r_old << "\n");

		measurements->poseFilter_.applyMeasurement(idx,H_old,r_old,R);
	}

}


