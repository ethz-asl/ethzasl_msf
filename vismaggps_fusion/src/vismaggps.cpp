/*
 * vismaggps.cpp
 *
 *  Created on: Sep 30, 2010
 *      Author: sweiss
 */

#include "vismaggps.h"
#include "vismaggps_measurements.h"

// measurement size
#define nVisMeas_ 6
#define nVisMagGPSMeas_ 14
//#define nVisMagGPSMeas_ 11
#define nGPSMeas_ 5	// safety mode
#define C_DELAY 0	//constant delay of camera measurement (added to that set in reconfigure gui)

void VisMagGPSHandler::subscribe(){

	ros::NodeHandle nh("sensor_fusion");
	subVisMeas_ = nh.subscribe("vision_pose", 1, &VisMagGPSHandler::visionCallback, this);
	subMagMeas_ = nh.subscribe("mag_dir", 1, &VisMagGPSHandler::magCallback, this);
	subGPSMeas_ = nh.subscribe("gps_pos", 1, &VisMagGPSHandler::gpsCallback, this);

	nh.param("/vismaggps_fusion/meas_noise1",n_zvq_,0.01);
	nh.param("/vismaggps_fusion/meas_noise2",n_zvp_,0.02);
	nh.param("/vismaggps_fusion/meas_noise3",n_zm_,0.002);
	nh.param("/vismaggps_fusion/meas_noise4",n_zgxy_,0.5);
	nh.param("/vismaggps_fusion/meas_noise5",n_zgz_,0.1);
	nh.param("/vismaggps_fusion/meas_noise6",n_zgv_,0.2);

	// setup: initial pos, att, of measurement sensor
	VisMagGPSMeasurements* customMeas = (VisMagGPSMeasurements*)(measurements);
	customMeas->poseFilter_.registerCallback(&VisMagGPSHandler::noiseConfig, this);
	customMeas->q_cv_ = Eigen::Quaternion<double>(1, 0, 0, 0);
	customMeas->p_vc_ = Eigen::Matrix<double, 3, 1>::Constant(0);
	customMeas->p_m_ = Eigen::Matrix<double, 3, 1>::Constant(0);
	customMeas->p_wg_ = Eigen::Matrix<double, 3, 1>::Constant(0);
	customMeas->v_wg_ = Eigen::Matrix<double, 2, 1>::Constant(0);

	MagBuff_.clear();
	GPSBuff_.clear();
	PTAMwatch_ = 0;
}


void VisMagGPSHandler::noiseConfig(sensor_fusion_core::Sensor_Fusion_CoreConfig& config, uint32_t level)
{
	if(level & sensor_fusion_core::Sensor_Fusion_Core_INIT_FILTER)
	{
		// clear measurement buffers at init step
		MagBuff_.clear();
		GPSBuff_.clear();
		PTAMwatch_ = 0;
	}
	this->n_zvq_ = config.meas_noise1;
	this->n_zvp_ = config.meas_noise2;
	this->n_zm_ = config.meas_noise3;
	this->n_zgxy_ = config.meas_noise4;
	this->n_zgz_ = config.meas_noise5;
	this->n_zgv_ = config.meas_noise6;
	this->setDELAY(config.delay);
}


void VisMagGPSHandler::magCallback(const geometry_msgs::Vector3StampedConstPtr & msg)
{
	MagMeas buffmag;
	buffmag.mag_[0]=msg->vector.x;
	buffmag.mag_[1]=msg->vector.y;
	buffmag.mag_[2]=msg->vector.z;
	buffmag.time_=msg->header.stamp.toSec();
	MagBuff_.push_back(buffmag);
}


void VisMagGPSHandler::gpsCallback(const vismaggps_fusion::GpsCustomCartesianConstPtr & msg)
{
	if(PTAMwatch_>5)	// we received 5 gps measurements and no vision measurements...PTAM is dead (?)
	{
		ROS_WARN_STREAM_THROTTLE(1,"MAV state estimaten runs in safety mode (GPS only)!!");
		State state_old;
		ros::Time time_old = msg->header.stamp;
		Sensor_Fusion_Core::MatrixXSd H_old = Eigen::Matrix<double, nGPSMeas_, nState_>::Constant(0);
		Eigen::VectorXd r_old = Eigen::Matrix<double, nGPSMeas_, 1>::Constant(0);
		Eigen::MatrixXd R = Eigen::Matrix<double, nGPSMeas_, nGPSMeas_>::Constant(0);

		z_gp_ = Eigen::Matrix<double, 3, 1>(msg->position.x, msg->position.y, msg->position.z); // change here for different topics
		z_gv_ = Eigen::Matrix<double, 2, 1>(msg->velocity_x, msg->velocity_y); // get here the gps velocities.... // change here for different topics

		Eigen::Matrix<double, nGPSMeas_, 1> buffvec;
		buffvec << n_zgxy_, n_zgxy_, n_zgz_, msg->velocity_covariance[0], msg->velocity_covariance[3];
		R = buffvec.asDiagonal();

		VisMagGPSMeasurements* customMeas = (VisMagGPSMeasurements*)(measurements);
		customMeas->p_wg_ = z_gp_;
		customMeas->v_wg_ = z_gv_;

		unsigned char idx = measurements->poseFilter_.getClosestState(&state_old, time_old, -DELAY_); //compensate for vision delay used in core...
		if (state_old.time_ == -1)
			return; /// no prediction made yet, EARLY ABORT
		Eigen::Matrix<double, 3, 3> C_q = state_old.q_.conjugate().toRotationMatrix();
		Eigen::Matrix<double, 3, 1> ew = state_old.w_m_ - state_old.b_w_;

		Eigen::Matrix<double, 3, 3> w_sk;
		w_sk << 0, -ew(2), ew(1), ew(2), 0, -ew(0), -ew(1), ew(0), 0;

		Eigen::Matrix<double, 3, 3> skewold_p;
		Eigen::Matrix<double, 3, 1> vecold_p;
		vecold_p = (state_old.p_ + C_q.transpose() * state_old.p_ig_);
		skewold_p << 0, -vecold_p(2), vecold_p(1),
				vecold_p(2), 0, -vecold_p(0),
				-vecold_p(1), vecold_p(0), 0;

		Eigen::Matrix<double, 3, 3> skewold_v;
		Eigen::Matrix<double, 3, 1> vecold_v;
		vecold_v = (state_old.v_ + C_q.transpose() * w_sk * state_old.p_ig_);
		skewold_v << 0, -vecold_v(2), vecold_v(1),
				vecold_v(2), 0, -vecold_v(0),
				-vecold_v(1), vecold_v(0), 0;

		Eigen::Matrix<double, 3, 3> pig_sk;
		pig_sk << 0, -state_old.p_ig_(2), state_old.p_ig_(1),
				state_old.p_ig_(2), 0, -state_old.p_ig_(0),
				-state_old.p_ig_(1), state_old.p_ig_(0), 0;

		Eigen::Matrix<double, 3, 3> wpig_sk;
		Eigen::Matrix<double, 3, 1> wpig_vec;
		wpig_vec = w_sk * state_old.p_ig_;
		wpig_sk << 0, -wpig_vec(2), wpig_vec(1),
				wpig_vec(2), 0, -wpig_vec(0),
				-wpig_vec(1), wpig_vec(0), 0;

		//		//measurements (kind of from mathematica...)
		//		q_vw*(vwi+Transpose[R1wi].skeww.pig)*L
		//		q_vw*(pwi+Transpose[R1wi].pig)*L

		// construct H matrix using H-blockx :-)
		H_old.block(0, 0, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
		H_old.block(0, 6, 3, 3) = -C_q.transpose() * pig_sk;
		H_old.block(0, 28, 3, 3) = C_q.transpose();

		Eigen::Matrix<double, 3, nState_> H_gpsvel = Eigen::Matrix<double, 3, nState_>::Constant(0);
		H_gpsvel.block(0, 3, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
		H_gpsvel.block(0, 6, 3, 3) = -C_q.transpose() * wpig_sk;
		H_gpsvel.block(0, 28, 3, 3) = C_q.transpose() * w_sk;
		H_old.block(3, 0, 2, nState_) = H_gpsvel.block(0, 0, 2, nState_); // only take xy vel measurements

//		H_old.block(5, 16, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity(); //q_ic
//		H_old.block(8, 19, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity(); //q_vw
//		H_old(11, 15) = 1; // scale

		Eigen::Matrix<double, 3, 1> gpsvelestim = (state_old.v_ + C_q.transpose() * w_sk * state_old.p_ig_);
		r_old.block(0, 0, 3, 1) = z_gp_ - (state_old.p_ + C_q.transpose() * state_old.p_ig_);
		r_old.block(3, 0, 2, 1) = z_gv_ - gpsvelestim.block(0, 0, 2, 1); // only take xy vel measurements
//		r_old.block(5, 0, 3, 1) = Eigen::Matrix<double,3,1>::Constant(0); //-state_old.q_wv_.vec() / state_old.q_wv_.w() * 2;
//		r_old.block(8, 0, 3, 1) = Eigen::Matrix<double,3,1>::Constant(0); //-state_old.q_ci_.vec() / state_old.q_ci_.w() * 2;
//		r_old(11) = 0; //1 - state_old.L_;

		measurements->poseFilter_.applyMeasurement(idx, H_old, r_old, R);
	}
	else
	{
		GPSMeas buffgps;
		buffgps.gp_(0,0)=msg->position.x;
		buffgps.gp_(1,0)=msg->position.y;
		buffgps.gp_(2,0)=msg->position.z;
		buffgps.gv_(0,0)=msg->velocity_x;
		buffgps.gv_(1,0)=msg->velocity_y;
		buffgps.time_=msg->header.stamp.toSec();
		GPSBuff_.push_back(buffgps);
		PTAMwatch_++;
	}
}


void VisMagGPSHandler::visionCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr & msg)
{
	bool hasmag=false, hasgps=false;
	MagMeas mag;
	GPSMeas gps;

	PTAMwatch_=0;	// PTAM is running...good

	static double prevtime = 0;
	static double difftime = 0;
	difftime = 0.9*difftime + 0.1*(msg->header.stamp.toSec()-C_DELAY-DELAY_-prevtime);

	if(prevtime==0)
	{
		prevtime=msg->header.stamp.toSec()-C_DELAY-DELAY_;
		return; // EARLY ABORT getting first vision measurement: init difftime (i.e. meas frequency)
	}

	unsigned int measidx = 0;
	double timedist = 1e100;
	double timenow = msg->header.stamp.toSec()-C_DELAY-DELAY_;
	if(MagBuff_.size()>0)
	{
		while (fabs(timenow-MagBuff_[measidx].time_)<timedist) // timedist decreases continuously until best point reached... then rises again
		{
			timedist = fabs(timenow-MagBuff_[measidx].time_);
			measidx++;
			if(!(measidx<MagBuff_.size()))
				break;
		}
		if(timedist<difftime/2)
		{
			hasmag=true;
			mag=MagBuff_[measidx-1];
			for(unsigned int i=0;i<measidx;i++)
				MagBuff_.erase(MagBuff_.begin());	//delete n times the first element...
		}
	}

	measidx = 0;
	timedist = 1e100;
	timenow = msg->header.stamp.toSec()-C_DELAY-DELAY_;
	if(GPSBuff_.size()>0)
	{
		while (fabs(timenow-GPSBuff_[measidx].time_)<timedist) // timedist decreases continuously until best point reached... then rises again
		{
			timedist = fabs(timenow-GPSBuff_[measidx].time_);
			measidx++;
			if(!(measidx<GPSBuff_.size()))
				break;
		}
		if(timedist<difftime/2)
		{
			hasgps=true;
			gps=GPSBuff_[measidx-1];
			for(unsigned int i=0;i<measidx;i++)
				GPSBuff_.erase(GPSBuff_.begin());	//delete n times the first element...
		}
	}

	z_vp_ = Eigen::Matrix<double,3,1>(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
	z_vq_ = Eigen::Quaternion<double>(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
	z_vp_ = -z_vq_.toRotationMatrix().transpose()*z_vp_;

	VisMagGPSMeasurements* customMeas = (VisMagGPSMeasurements*)(measurements);
	customMeas->p_vc_ = z_vp_;
	customMeas->q_cv_ = z_vq_;

	if(hasmag)
	{
		z_m_ = Eigen::Matrix<double,3,1>(mag.mag_[0],mag.mag_[1],mag.mag_[2]);
		z_m_.normalize();
	}
	if(hasgps)
	{
		z_gp_ = Eigen::Matrix<double,3,1>(gps.gp_[0], gps.gp_[1], gps.gp_[2]);
		z_gv_ = Eigen::Matrix<double,2,1>(gps.gv_[0], gps.gv_[1]);
		customMeas->p_wg_ = z_gp_;
		customMeas->v_wg_ = z_gv_;
	}

	State state_old;
	ros::Time time_old = msg->header.stamp;
	unsigned char idx = measurements->poseFilter_.getClosestState(&state_old,time_old,C_DELAY);
	if (state_old.time_ == -1)
		return;		/// no prediction made yet, EARLY ABORT


	Sensor_Fusion_Core::MatrixXSd H_old = Eigen::Matrix<double,nVisMeas_,nState_>::Constant(0);
	Eigen::VectorXd r_old(nVisMeas_);

	Eigen::MatrixXd R = Eigen::Matrix<double,nVisMeas_,nVisMeas_>::Constant(0);
	R.block(0,0,6,6) = Eigen::Matrix<double,6,6>(&msg->pose.covariance[0]);
	Eigen::Matrix<double,nVisMeas_,1> buffvec = R.diagonal();
	R = Eigen::Matrix<double,nVisMeas_,nVisMeas_>::Constant(0);
	R = buffvec.asDiagonal();


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
	q_err = (state_old.q_wv_*state_old.q_*state_old.q_ci_).conjugate()*z_vq_.conjugate();
	r_old.block(0,0,3,1) = z_vp_ - (state_old.p_vw_+C_wv.transpose()*(state_old.p_ + C_q.transpose()*state_old.p_ic_))*state_old.L_;
	r_old.block(3,0,3,1) = q_err.vec()/q_err.w()*2;


	if(hasmag && hasgps)
	{
		Sensor_Fusion_Core::MatrixXSd Htot = Eigen::Matrix<double,nVisMagGPSMeas_,nState_>::Constant(0);
		Eigen::VectorXd rtot = Eigen::Matrix<double,nVisMagGPSMeas_,1>::Constant(0);
		Eigen::MatrixXd Rtot = Eigen::Matrix<double,nVisMagGPSMeas_,nVisMagGPSMeas_>::Constant(0);
		// copy vision measurement
		Htot.block(0,0,nVisMeas_,nState_) = H_old;
		rtot.block(0,0,nVisMeas_,1) = r_old;
		Rtot.block(0,0,nVisMeas_,nVisMeas_) = R;

/////////////////// fill in mag measurement ////////////////////////////////////////////////////////////////

		Eigen::Matrix<double,3,1> magRvec;
		magRvec  << n_zm_,n_zm_,n_zm_;
		Rtot.block(nVisMeas_,nVisMeas_,3,3) = magRvec.asDiagonal();
		customMeas->p_m_ = z_m_;

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
		Htot.block(nVisMeas_,6,3,3) = C_mi*mw_sk1;
		Htot.block(nVisMeas_,25,3,3) = mw_sk2;
		Htot.block(nVisMeas_,34,3,1) = C_mi*C_q*vec_m_dalpha;
		Htot.block(nVisMeas_,35,3,1) =C_mi*C_q*vec_m_dbeta;

		rtot.block(nVisMeas_,0,3,1) = z_m_ - C_mi*C_q*(vec_m);

/////////////////// fill in gps measurement ////////////////////////////////////////////////////////////////

		Eigen::Matrix<double,5,1> gpsRvec;
		gpsRvec  << n_zgxy_,n_zgxy_,n_zgz_,n_zgv_,n_zgv_;
		Rtot.block(nVisMeas_+3,nVisMeas_+3,5,5) = gpsRvec.asDiagonal();

		Eigen::Matrix<double,3,1> ew = state_old.w_m_-state_old.b_w_;

		Eigen::Matrix<double,3,3> w_sk;
		w_sk << 0, -ew(2), ew(1)
				,ew(2), 0, -ew(0)
				,-ew(1), ew(0), 0;

		Eigen::Matrix<double,3,3> skewold_p;
		Eigen::Matrix<double,3,1> vecold_p;
		vecold_p = (state_old.p_+C_q.transpose()*state_old.p_ig_);
		skewold_p << 0, -vecold_p(2), vecold_p(1)
				,vecold_p(2), 0, -vecold_p(0)
				,-vecold_p(1), vecold_p(0), 0;

		Eigen::Matrix<double,3,3> skewold_v;
		Eigen::Matrix<double,3,1> vecold_v;
		vecold_v = (state_old.v_+C_q.transpose()*w_sk*state_old.p_ig_);
		skewold_v << 0, -vecold_v(2), vecold_v(1)
				,vecold_v(2), 0, -vecold_v(0)
				,-vecold_v(1), vecold_v(0), 0;

		Eigen::Matrix<double,3,3> pig_sk;
		pig_sk << 0, -state_old.p_ig_(2), state_old.p_ig_(1)
				,state_old.p_ig_(2), 0, -state_old.p_ig_(0)
				,-state_old.p_ig_(1), state_old.p_ig_(0), 0;

		Eigen::Matrix<double,3,3> wpig_sk;
		Eigen::Matrix<double,3,1> wpig_vec;
		wpig_vec = w_sk*state_old.p_ig_;
		wpig_sk << 0, -wpig_vec(2), wpig_vec(1)
				,wpig_vec(2), 0, -wpig_vec(0)
				,-wpig_vec(1), wpig_vec(0), 0;

		// construct H matrix using H-blockx :-)
		Htot.block(nVisMeas_+3,0,3,3) = C_wv.transpose();
		Htot.block(nVisMeas_+3,6,3,3) = -C_wv.transpose()*C_q.transpose()*pig_sk;
		Htot.block(nVisMeas_+3,16,3,3) = -C_wv.transpose()*skewold_p;
		Htot.block(nVisMeas_+3,22,3,3) = C_wv.transpose()*C_q.transpose();

		Eigen::Matrix<double,3,nState_> H_gpsvel = Eigen::Matrix<double,3,nState_>::Constant(0);
		H_gpsvel.block(0,3,3,3) = C_wv.transpose();
		H_gpsvel.block(0,6,3,3) = -C_wv.transpose()*C_q.transpose()*wpig_sk;
		H_gpsvel.block(0,16,3,3) = -C_wv.transpose()*skewold_v;
		H_gpsvel.block(0,22,3,3) = C_wv.transpose()*C_q.transpose()*w_sk;
		Htot.block(nVisMeas_+3+3,0,2,nState_) = H_gpsvel.block(0,0,2,nState_); // only take xy vel measurements

		Eigen::Matrix<double,3,1> gpsvelest = C_wv.transpose()*(state_old.v_ + C_q.transpose()*w_sk*state_old.p_ig_);
		rtot.block(nVisMeas_+3,0,3,1) = z_gp_ - C_wv.transpose()*(state_old.p_ + C_q.transpose()*state_old.p_ig_);
		rtot.block(nVisMeas_+3+3,0,2,1) = z_gv_ - gpsvelest.block(0,0,2,1);;	// only take xy vel measurements


//		Eigen::Matrix<double,5,1> gpsRvec;
//		gpsRvec  << n_zgxy_,n_zgxy_,n_zgz_,n_zgv_,n_zgv_;
//		Rtot.block(nVisMeas_,nVisMeas_,5,5) = gpsRvec.asDiagonal();
//
//		Eigen::Matrix<double,3,1> ew = state_old.w_m_-state_old.b_w_;
//
//		Eigen::Matrix<double,3,3> w_sk;
//		w_sk << 0, -ew(2), ew(1)
//				,ew(2), 0, -ew(0)
//				,-ew(1), ew(0), 0;
//
//		Eigen::Matrix<double,3,3> skewold_p;
//		Eigen::Matrix<double,3,1> vecold_p;
//		vecold_p = (state_old.p_+C_q.transpose()*state_old.p_ig_);
//		skewold_p << 0, -vecold_p(2), vecold_p(1)
//				,vecold_p(2), 0, -vecold_p(0)
//				,-vecold_p(1), vecold_p(0), 0;
//
//		Eigen::Matrix<double,3,3> skewold_v;
//		Eigen::Matrix<double,3,1> vecold_v;
//		vecold_v = (state_old.v_+C_q.transpose()*w_sk*state_old.p_ig_);
//		skewold_v << 0, -vecold_v(2), vecold_v(1)
//				,vecold_v(2), 0, -vecold_v(0)
//				,-vecold_v(1), vecold_v(0), 0;
//
//		Eigen::Matrix<double,3,3> pig_sk;
//		pig_sk << 0, -state_old.p_ig_(2), state_old.p_ig_(1)
//				,state_old.p_ig_(2), 0, -state_old.p_ig_(0)
//				,-state_old.p_ig_(1), state_old.p_ig_(0), 0;
//
//		Eigen::Matrix<double,3,3> wpig_sk;
//		Eigen::Matrix<double,3,1> wpig_vec;
//		wpig_vec = w_sk*state_old.p_ig_;
//		wpig_sk << 0, -wpig_vec(2), wpig_vec(1)
//				,wpig_vec(2), 0, -wpig_vec(0)
//				,-wpig_vec(1), wpig_vec(0), 0;
//
//		// construct H matrix using H-blockx :-)
//		Htot.block(nVisMeas_,0,3,3) = C_wv.transpose();
//		Htot.block(nVisMeas_,6,3,3) = -C_wv.transpose()*C_q.transpose()*pig_sk;
//		Htot.block(nVisMeas_,16,3,3) = -C_wv.transpose()*skewold_p;
//		Htot.block(nVisMeas_,22,3,3) = C_wv.transpose()*C_q.transpose();
//
//		Eigen::Matrix<double,3,nState_> H_gpsvel = Eigen::Matrix<double,3,nState_>::Constant(0);
//		H_gpsvel.block(0,3,3,3) = C_wv.transpose();
//		H_gpsvel.block(0,6,3,3) = -C_wv.transpose()*C_q.transpose()*wpig_sk;
//		H_gpsvel.block(0,16,3,3) = -C_wv.transpose()*skewold_v;
//		H_gpsvel.block(0,22,3,3) = C_wv.transpose()*C_q.transpose()*w_sk;
//		Htot.block(nVisMeas_+3,0,2,nState_) = H_gpsvel.block(0,0,2,nState_); // only take xy vel measurements
//
//		Eigen::Matrix<double,3,1> gpsvelest = C_wv.transpose()*(state_old.v_ + C_q.transpose()*w_sk*state_old.p_ig_);
//		rtot.block(nVisMeas_,0,3,1) = z_gp_ - C_wv.transpose()*(state_old.p_ + C_q.transpose()*state_old.p_ig_);
//		rtot.block(nVisMeas_+3,0,2,1) = z_gv_ - gpsvelest.block(0,0,2,1);;	// only take xy vel measurements

		measurements->poseFilter_.applyMeasurement(idx,Htot,rtot,Rtot,0.2);
	}
	else
		measurements->poseFilter_.applyMeasurement(idx,H_old,r_old,R);

	prevtime = state_old.time_;

}

