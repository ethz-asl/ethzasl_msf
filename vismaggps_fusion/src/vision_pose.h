/*
 * vision_pose.h
 *
 *  Created on: Sep 30, 2010
 *      Author: sweiss
 */

#ifndef VISION_POSE_H
#define VISION_POSE_H

#include <sensor_fusion_core/measurement.h>

class VisionPoseHandler: public MeasurementHandler
{
	// measurements
	Eigen::Quaternion<double> z_q_;
	Eigen::Matrix<double, 3, 1> z_p_;
	double n_zp_, n_zq_;

	ros::Subscriber subMeasurement_;
	void subscribe();
	void measurementCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr & msg);
//	void executeMeasurement(char* msg, unsigned char idx);
	void noiseConfig(sensor_fusion_core::Sensor_Fusion_CoreConfig& config, uint32_t level);

public:
	VisionPoseHandler(Measurements* meas):MeasurementHandler(meas){subscribe();}
};

#endif /* VISION_POSE_H */
