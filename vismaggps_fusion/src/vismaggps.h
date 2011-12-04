/*
 * vismaggps.h
 *
 *  Created on: Sep 30, 2010
 *      Author: sweiss
 */

#ifndef VISMAGGPS_H
#define VISMAGGPS_H

#define nMeasBuff 256

#include <sensor_fusion_core/measurement.h>
#include <vismaggps_fusion/GpsCustomCartesian.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <Eigen/StdVector>	// include this to use std::vectors with eigen...


class MagMeas
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Eigen::Matrix<double, 3, 1> mag_;
	Eigen::Matrix<double, 3, 1> n_mag_;
	double time_;
};

class GPSMeas
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Eigen::Matrix<double, 3, 1> gp_;
	Eigen::Matrix<double, 3, 1> n_gp_;
	Eigen::Matrix<double, 2, 1> gv_;
	Eigen::Matrix<double, 2, 1> n_gv_;
	double time_;
};

class VisMagGPSHandler: public MeasurementHandler
{
	// measurements
	Eigen::Quaternion<double> z_vq_;
	Eigen::Matrix<double, 3, 1> z_vp_;
	Eigen::Matrix<double, 3, 1> z_m_;
	Eigen::Matrix<double, 3, 1> z_gp_;
	Eigen::Matrix<double, 2, 1> z_gv_;
	double n_zvq_, n_zvp_, n_zm_, n_zgxy_, n_zgz_, n_zgv_;

	double DELAY_;	/// const time delay of measurements


	std::vector<MagMeas,Eigen::aligned_allocator<MagMeas> > MagBuff_;
	std::vector<GPSMeas,Eigen::aligned_allocator<GPSMeas> > GPSBuff_;

	// PTAM watchdog and init sequence
	int PTAMwatch_;
	int INITsequence_;


	ros::Subscriber subVisMeas_;
	ros::Subscriber subMagMeas_;
	ros::Subscriber subGPSMeas_;
	void subscribe();

	void visionCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr & msg);
	void magCallback(const geometry_msgs::Vector3StampedConstPtr & msg);
	void gpsCallback(const vismaggps_fusion::GpsCustomCartesianConstPtr & msg);
	void noiseConfig(sensor_fusion_core::Sensor_Fusion_CoreConfig& config, uint32_t level);

	void setDELAY(double val) {DELAY_ = val;};

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	VisMagGPSHandler(Measurements* meas):MeasurementHandler(meas){subscribe();}
};

#endif /* VISMAGGPS_H */
