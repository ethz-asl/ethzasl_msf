/*
 * gps_pos.h
 *
 *  Created on: Sep 30, 2010
 *      Author: sweiss
 */

#ifndef GPS_POS_H_
#define GPS_POS_H_

#include <sensor_fusion_core/measurement.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>



class GPSPosHandler: public MeasurementHandler
{
	// measurements
	Eigen::Matrix<double, 3, 1> z_p_;
	double n_zp_;

	ros::Subscriber subMeasurement_;
	void subscribe();
//	void measurementCallback(const geometry_msgs::PointStampedConstPtr & msg);
	void measurementCallback(const geometry_msgs::TransformStampedConstPtr & msg);
	void noiseConfig(sensor_fusion_core::Sensor_Fusion_CoreConfig& config, uint32_t level);

public:
	GPSPosHandler(Measurements* meas):MeasurementHandler(meas){subscribe();}
};

#endif /* GPS_POS_H_ */
