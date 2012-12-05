/*
 * viconpos_measurement.hpp
 *
 *  Created on: Nov 13, 2012
 *      Author: slynen
 */

#ifndef VICONPOS_MEASUREMENT_HPP_
#define VICONPOS_MEASUREMENT_HPP_

#include <msf_core/msf_measurement.hpp>
#include <msf_core/msf_core.hpp>

namespace{
enum{
	nMeasurements = 9
};
}

struct ViconMeasurement:public msf_core::MSF_Measurement<geometry_msgs::TransformStamped, nMeasurements>{
private:
	virtual void makeFromSensorReadingImpl(geometry_msgs::TransformStampedConstPtr msg, bool fixedCovariance){
		// get measurements
		z_p_ = Eigen::Matrix<double,3,1>(msg->transform.translation.x, msg->transform.translation.y, msg->transform.translation.z);

		R_.setZero();

		if (!fixedCovariance)  // take covariance from sensor
		{
			//		meas.R.block(0,0,3,3) = Eigen::Matrix<double,3,3>(&msg->covariance[0]);
			//		Eigen::Matrix<double,6,1> buffvec = Eigen::Matrix<double,6,1>::Constant(1e-6);
			//		meas.R.block(3,3,6,6) = buffvec.asDiagonal(); // measurement noise for q_vw, q_ci
		}
		else  // alternatively take fix covariance from reconfigure GUI
		{
			const double s_zp = n_zp_ * n_zp_;
			R_ = (Eigen::Matrix<double, nMeasurements, 1>() << s_zp, s_zp, s_zp, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6).finished().asDiagonal();
		}
	}
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	Eigen::Matrix<double, 3, 1> z_p_;
	double n_zp_;

	typedef msf_core::EKFState state_T;
	~ViconMeasurement(){};
	ViconMeasurement(double n_zp){
		n_zp_ = n_zp;
	};
	virtual void apply(boost::shared_ptr<state_T> state, msf_core::MSF_Core& core){
		// init variables
		Eigen::Matrix<double,nMeasurements,msf_core::MSF_Core::nErrorStatesAtCompileTime> H_old;
		Eigen::Matrix<double, nMeasurements, 1> r_old;

		H_old.setZero();

		if (state->time_ == -1){
			ROS_WARN_STREAM("apply vicon update was called with an invalid state");
			return;	// // early abort // //
		}

		const state_T& state_old = *state; //to overload for const getters

		// get rotation matrices
		Eigen::Matrix<double,3,3> C_wv = state_old.get<msf_core::q_wv_>().conjugate().toRotationMatrix();
		Eigen::Matrix<double,3,3> C_q = state_old.get<msf_core::q_>().conjugate().toRotationMatrix();
//		Eigen::Matrix<double,3,3> C_ci = state_old.get<msf_core::q_ci_>().conjugate().toRotationMatrix();

		// preprocess for elements in H matrix
		Eigen::Matrix<double,3,1> vecold;
		vecold = (state_old.get<msf_core::p_>()+C_q.transpose()*state_old.get<msf_core::p_ci_>())*state_old.get<msf_core::L_>();
		Eigen::Matrix<double,3,3> skewold = skew(vecold);

		Eigen::Matrix<double,3,3> pci_sk = skew(state_old.get<msf_core::p_ci_>());

		ROS_INFO_STREAM("timediff to state "<<std::fabs(state->time_-time_));
		ROS_INFO_STREAM("apply meas scale = "<<state_old.get<msf_core::L_>()(0));

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

		// call update step in base class
		MSF_Measurement::calculateAndApplyCorrection(state, core, H_old,r_old,R_);

	}
};



#endif /* VICONPOS_MEASUREMENT_HPP_ */
