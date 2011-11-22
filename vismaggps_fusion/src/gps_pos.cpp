/*
 * gps_pos.cpp
 *
 *  Created on: Sep 30, 2010
 *      Author: sweiss
 */

#include "gps_pos.h"
#include "vismaggps_measurements.h"

// measurement size
#define nMeas_ 3

void GPSPosHandler::subscribe(){

	ros::NodeHandle nh("pose_filter");
	subMeasurement_ = nh.subscribe("gps_pos", 1, &GPSPosHandler::measurementCallback, this);

	nh.param("/leica_filter/meas_noise1",n_zp_,0.0001);

	VisMagGPSMeasurements* customMeas = (VisMagGPSMeasurements*)(measurements);
	customMeas->poseFilter_.registerCallback(&GPSPosHandler::noiseConfig, this);
	customMeas->p_wg_ = Eigen::Matrix<double, 3, 1>::Constant(0);

}

void GPSPosHandler::noiseConfig(sensor_fusion_core::Sensor_Fusion_CoreConfig& config, uint32_t level){
	//	if(level & sensor_fusion_core::Sensor_Fusion_Core_MISC)
	//	{
	this->n_zp_ = config.meas_noise1;
	//	}
}

//void LeicaPoseHandler::measurementCallback(const geometry_msgs::PointStampedConstPtr & msg)
void GPSPosHandler::measurementCallback(const geometry_msgs::TransformStampedConstPtr & msg)
{
//	ROS_INFO_STREAM("measurement received \n");

	State state_old;
	ros::Time time_old = msg->header.stamp;
	Sensor_Fusion_Core::MatrixXSd H_old = Eigen::Matrix<double,nMeas_,nState_>::Constant(0);
	Eigen::VectorXd r_old(nMeas_);

//	z_p_ = Eigen::Matrix<double,3,1>(msg->point.x, msg->point.y, msg->point.z);
	z_p_ = Eigen::Matrix<double,3,1>(msg->transform.translation.x, msg->transform.translation.y, msg->transform.translation.z);

	Eigen::MatrixXd R = Eigen::Matrix<double,nMeas_,nMeas_>::Constant(0);
	Eigen::Matrix<double,nMeas_,1> buffvec;
	buffvec  << n_zp_,n_zp_,n_zp_;
	R = buffvec.asDiagonal();

	VisMagGPSMeasurements* customMeas = (VisMagGPSMeasurements*)(measurements);
	customMeas->p_wg_ = z_p_;

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

		Eigen::Matrix<double,3,3> pig_sk;
		pig_sk << 0, -state_old.p_ig_(2), state_old.p_ig_(1)
				,state_old.p_ig_(2), 0, -state_old.p_ig_(0)
				,-state_old.p_ig_(1), state_old.p_ig_(0), 0;

		// construct H matrix using H-blockx :-)
		H_old.block(0,0,3,3) = Eigen::Matrix<double,3,3>::Identity();
		H_old.block(0,6,3,3) = -C_q.transpose()*pig_sk;
		H_old.block(0,28,3,3) = C_q.transpose();

		r_old.block(0,0,3,1) = z_p_ - (state_old.p_ + C_q.transpose()*state_old.p_ig_);

//		ROS_INFO_STREAM("measurements: " << "\n"
//				<< "L: " << state_old.L_ << "\n"
//				<< "rmeas: " << r_old << "\n");

		measurements->poseFilter_.applyMeasurement(idx,H_old,r_old,R);
	}

}


