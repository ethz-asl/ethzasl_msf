/*
 * Sensor_Fusion_Core.h
 *
 *  Created on: Sep 30, 2010
 *      Author: sweiss
 */

#ifndef SENSOR_FUSION_CORE_H_
#define SENSOR_FUSION_CORE_H_


#include <Eigen/Eigen>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_fusion_core/DoubleArrayStamped.h>

#include <sensor_fusion_core/Sensor_Fusion_CoreConfig.h>

#include <sensor_fusion_core/ext_imu.h>
#include <sensor_fusion_core/ext_state.h>
#include <sensor_fusion_core/ext_ekf.h>

#include <boost/thread/mutex.hpp>

#include <vector>

#define nState_ 36 				// error state
#define nStateBuffer_ 256		// size of unsigned char, do not change!
#define HLI_EKF_STATE_SIZE 16 	// number of states exchanged with external propagation. Here: p,v,q,bw,bw=16

typedef dynamic_reconfigure::Server<sensor_fusion_core::Sensor_Fusion_CoreConfig> ReconfigureServer;


struct State
{
	Eigen::Matrix<double, 3, 1> p_;
	Eigen::Matrix<double, 3, 1> v_;
	Eigen::Quaternion<double> q_;
	Eigen::Matrix<double, 3, 1> b_w_;
	Eigen::Matrix<double, 3, 1> b_a_;
	double L_;
	Eigen::Quaternion<double> q_wv_;
	Eigen::Quaternion<double> q_ci_;
	Eigen::Matrix<double, 3, 1> p_ic_;
	Eigen::Quaternion<double> q_mi_;
	Eigen::Matrix<double, 3, 1> p_ig_;
	Eigen::Matrix<double, 3, 1> p_vw_;
	double alpha_;	// mag vec elevation
	double beta_;	// mag vec azimuth

	Eigen::Matrix<double, nState_, nState_> P_;
	Eigen::Matrix<double,3,1> w_m_;
	Eigen::Quaternion<double> q_int_;	// this is the integrated ang. vel. no corrections applied, to use for delta rot in external algos...
	Eigen::Matrix<double,3,1> a_m_;
	double time_;
};

struct Meas
{
//	void (*measfunc_)(char*, unsigned char);
//	char* message_;
	unsigned int msize_;
	unsigned char stateidx_;
	double time_;
};

class Sensor_Fusion_Core {
public:

	typedef Eigen::Matrix<double,Eigen::Dynamic,nState_> MatrixXSd;

	//dynamic reconfigure stuff
	template<class T>
	void registerCallback(void(T::*cb_func)(sensor_fusion_core::Sensor_Fusion_CoreConfig& config, uint32_t level), T* p_obj){
		callbacks_.push_back(boost::bind(cb_func, p_obj, _1, _2));};

private:

	boost::mutex ekf_mutex_;

	const static int nFullState_ = 40;	// complete state
	const static int nBuff_ = 30;		// buffer size for median q_vw
	const static int nMaxCorr_ = 50;	// number of IMU measurements buffered for time correction actions
	const static int QualityThres_ = 1e3;

	Eigen::Matrix<double, nState_, nState_> Fd_;
	Eigen::Matrix<double, nState_, nState_> Qd_;
	MatrixXSd H_;
	Eigen::MatrixXd S_;
	Eigen::MatrixXd K_;
	Eigen::Matrix<double, 3, 1> g_;

	// state variables
	State StateBuffer_[nStateBuffer_];
	unsigned char idx_state_;	// pointer to state buffer at most recent state
	unsigned char idx_P_;		// pointer to state buffer at P latest propagated
	unsigned char idx_time_;	// pointer to state buffer at a specific time
	unsigned char idx_meas_;	// pointer to measurement buffer with the newest measurement

	// measurements buffer
	std::list<Meas> MeasBuffer_;	// unsigned char is the state index in StateBuffer, newest meas is at beginning!


	int qvw_inittimer_;
	Eigen::Matrix<double,nBuff_,4> qbuff_;

	Eigen::Matrix<double,nState_,1> correction_;

	double n_a_;	/// acc noise
	double n_ba_;	/// bias acc noise
	double n_w_;	/// gyro noise
	double n_bw_;	/// bias gyro noise
	double n_L_;	/// scale drift noise
	double n_qwv_;	/// vision world attitude drift noise
	double n_qci_;	/// imu-cam attitude drift noise
	double n_pic_;	/// imu-cam position drift noise
	double n_qmi_;	/// imu-mag attitude drift noise
	double n_pig_;	/// imu-gps position drift noise
	double n_pvw_;	/// vision-world position drift noise
	double n_alpha_;	/// mag vec leveation drift noise
	double n_beta_;	/// mag vec azimuth drift noise
	double DELAY_;	/// const time delay of measurements

	Eigen::Matrix<double, 3, 3> R_IW_; 		/// Rot IMU->World
	Eigen::Matrix<double, 3, 3> R_CI_;  	/// Rot Camera->IMU
	Eigen::Matrix<double, 3, 3> R_WV_;  	/// Rot World->Vision
	Eigen::Matrix<double, 3, 3> R_MI_;  	/// Rot IMU->Mag

	bool fixedScale_;
	bool fixedBias_;
	bool predictionMade_;
	bool initialized_;

	enum{NO_UP,GOOD_UP, FUZZY_UP};

	ros::Publisher pubState_;
	sensor_fusion_core::DoubleArrayStamped msgState_;

	ros::Publisher pubPose_;
	geometry_msgs::PoseWithCovarianceStamped msgPose_;

	ros::Publisher pubPoseCrtl_;
	sensor_fusion_core::ext_state msgPoseCtrl_;

	ros::Publisher pubCorrect_;
	sensor_fusion_core::ext_ekf msgCorrect_;

	ros::Subscriber subState_;
	ros::Subscriber subImu_;

	sensor_fusion_core::ext_ekf hl_state_buf_;

	// dynamic reconfigure stuff
	ReconfigureServer *reconfServer_;
	typedef boost::function<void(sensor_fusion_core::Sensor_Fusion_CoreConfig& config, uint32_t level)> CallbackType;
	std::vector<CallbackType> callbacks_;

	void propagateState(const double dt);
	void predictProcessCovariance(const double dt);
	void propPToIdx(unsigned char idx);
//	void imuCallback(const sfly_msgs::ext_ekfConstPtr & msg);	// real
//	void imuCallback(const sfly_msgs::ext_imuConstPtr & msg);	// debug, state prop here, old version
//	void imuCallback(const sensor_fusion_core::ext_imuConstPtr & msg);	// debug, state prop here, old version
	void imuCallback(const sensor_msgs::ImuConstPtr & msg);	// debug, state prop here, old version
	void stateCallback(const sensor_fusion_core::ext_ekfConstPtr & msg);
	void Config(sensor_fusion_core::Sensor_Fusion_CoreConfig &config, uint32_t level);
	void DynConfig(sensor_fusion_core::Sensor_Fusion_CoreConfig &config, uint32_t level);

public:

	void initialize(Eigen::Matrix<double, 3, 1> p, Eigen::Matrix<double, 3, 1> v,Eigen::Quaternion<double> q,
			Eigen::Matrix<double, 3, 1> b_w,Eigen::Matrix<double, 3, 1> b_a,double L,Eigen::Quaternion<double> q_wv,
			Eigen::Matrix<double, nState_, nState_> P,
			Eigen::Matrix<double, 3, 1> w_m,Eigen::Matrix<double, 3, 1> a_m,
			Eigen::Matrix<double, 3, 1> g, Eigen::Quaternion<double> q_ci, Eigen::Matrix<double, 3, 1> p_ic,
			Eigen::Quaternion<double> q_mi,  Eigen::Matrix<double, 3, 1> p_ig, Eigen::Matrix<double, 3, 1> p_vw,
			double alpha, double beta);

	bool applyMeasurement(unsigned char idx_delaystate, const MatrixXSd& H_delayed, const Eigen::VectorXd& res_delayed, const Eigen::MatrixXd& R_delayed,double fuzzythres=0.1);
//	bool registerMeasurement(void (*measfunc)(char*, unsigned char), char* message, unsigned int msize, unsigned char stateidx,ros::Time tstamp);
	unsigned char getClosestState(State* timestate, ros::Time tstamp, double delay=0.00);
	bool getStateAtIdx(State* timestate, unsigned char idx);


	int stateSize(){return nState_;};
	void setFixedScale(bool fixedScale) {fixedScale_ = fixedScale;};
	void setFixedBias(bool fixedBias) {fixedBias_ = fixedBias;};
	void setNoiseAcc(double val) {n_a_ = val;};
	void setNoiseAccBias(double val) {n_ba_ = val;};
	void setNoiseGyr(double val) {n_w_ = val;};
	void setNoiseGyrBias(double val) {n_bw_ = val;};
	void setNoiseScale(double val) {n_L_ = val;};
	void setNoiseWV(double val) {n_qwv_ = val;};
	void setNoiseQCI(double val) {n_qci_ = val;};
	void setNoisePIC(double val) {n_pic_ = val;};
	void setNoiseQMI(double val) {n_qmi_ = val;};
	void setNoisePIG(double val) {n_pig_ = val;};
	void setNoisePVW(double val) {n_pvw_ = val;};
	void setNoiseALPHA(double val) {n_alpha_ = val;};
	void setNoiseBETA(double val) {n_beta_ = val;};
	void setDELAY(double val) {DELAY_ = val;};

	Sensor_Fusion_Core();
	~Sensor_Fusion_Core();

private:
	double getMedian(Eigen::Matrix<double,nBuff_,1> data)
	{
		std::vector<double> mediandistvec;
		for(int i=0; i<nBuff_; ++i)
			mediandistvec[i]=data(i);

		if(mediandistvec.size()>0)
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

};


#endif /* SENSOR_FUSION_CORE_H_ */
