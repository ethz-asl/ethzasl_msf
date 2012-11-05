/*
 * mag_dir.h
 *
 *  Created on: Sep 30, 2010
 *      Author: sweiss
 */

#ifndef MAG_DIR_H_
#define MAG_DIR_H_

#include <sensor_fusion_core/measurement.h>
#include <geometry_msgs/PointStamped.h>



class MagDirHandler: public MeasurementHandler
{
	// measurements
	Eigen::Matrix<double, 3, 1> z_m_;
	double n_zm_;

	ros::Subscriber subMeasurement_;
	void subscribe();
	void measurementCallback(const geometry_msgs::PointStampedConstPtr & msg);
	void noiseConfig(sensor_fusion_core::Sensor_Fusion_CoreConfig& config, uint32_t level);

public:
	MagDirHandler(Measurements* meas):MeasurementHandler(meas){subscribe();}
};

#endif /* MAG_DIR_H_ */
