/*

Copyright (c) 2010, Stephan Weiss, ASL, ETH Zurich, Switzerland
You can contact the author at <stephan dot weiss at ieee dot org>
Copyright (c) 2012, Simon Lynen, ASL, ETH Zurich, Switzerland
You can contact the author at <slynen at ethz dot ch>

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
 * Neither the name of ETHZ-ASL nor the
names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETHZ-ASL BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */

#include <msf_core/eigen_utils.h>

PoseSensorHandler::PoseSensorHandler(msf_core::MSF_SensorManager& meas) :
SensorHandler(meas)
{
	ros::NodeHandle pnh("~");
	pnh.param("measurement_world_sensor", measurement_world_sensor_, true);
	pnh.param("use_fixed_covariance", use_fixed_covariance_, false);

	ROS_INFO_COND(measurement_world_sensor_, "interpreting measurement as sensor w.r.t. world");
	ROS_INFO_COND(!measurement_world_sensor_, "interpreting measurement as world w.r.t. sensor (e.g. ethzasl_ptam)");

	ROS_INFO_COND(use_fixed_covariance_, "using fixed covariance");
	ROS_INFO_COND(!use_fixed_covariance_, "using covariance from sensor");

	subscribe();
}

void PoseSensorHandler::subscribe()
{
	ros::NodeHandle nh("msf_core");
	subPoseWithCovarianceStamped_ = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("pose_with_covariance", 1, &PoseSensorHandler::measurementCallback, this);
	subTransformStamped_ = nh.subscribe<geometry_msgs::TransformStamped>("transform", 1, &PoseSensorHandler::measurementCallback, this);
	subPoseStamped_ = nh.subscribe<geometry_msgs::PoseStamped>("pose", 1, &PoseSensorHandler::measurementCallback, this);

	//TODO: we might need to implement that for initial testing
	//measurements->msf_core_.registerCallback(&PoseSensorHandler::noiseConfig, this);

	nh.param("meas_noise1", n_zp_, 0.01);	// default position noise is for ethzasl_ptam
	nh.param("meas_noise2", n_zq_, 0.02);	// default attitude noise is for ethzasl_ptam

	dynamic_cast<msf_core::MSF_SensorManagerROS&>(manager_).registerCallback(&PoseSensorHandler::noiseConfig, this);
	dynamic_cast<msf_core::MSF_SensorManagerROS&>(manager_).registerCallback(&PoseSensorHandler::dynConfig, this);

}

void PoseSensorHandler::noiseConfig(msf_core::MSF_CoreConfig& config, uint32_t level)
{
	//	if(level & msf_core::MSF_Core_MISC)
	//	{
	this->n_zp_ = config.meas_noise1;
	this->n_zq_ = config.meas_noise2;
	//	}
}

void PoseSensorHandler::dynConfig(msf_core::MSF_CoreConfig &config, uint32_t level){
	if(level & msf_core::MSF_Core_INIT_FILTER)
	{
		manager_.init(config.scale_init);
		config.init_filter = false;
	}
	else if(level & msf_core::MSF_Core_SET_HEIGHT)
	{
		if(z_p_.norm()==0)
		{
			ROS_WARN_STREAM("No measurements received yet to initialize position - using scale factor " << config.scale_init << " for init");
			manager_.init(config.scale_init);
		}
		else
		{
			manager_.init(z_p_[2]/config.height);
			ROS_WARN_STREAM("init filter (set scale to: " << z_p_[2]/config.height << ")");
		}
		config.set_height = false;
	}
	else if(level & msf_core::MSF_Core_SET_PRESS)
	{
		ROS_ERROR_STREAM("PRESSURE SCALE INIT NOT IMPLEMENTED AT THE MOMENT");
//		dynamic_cast<PoseSensorManager&>(manager_).init_scale(config.scale_init)
//		config.set_pressure_height = false;
	}
}

void PoseSensorHandler::measurementCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr & msg)
{
	boost::shared_ptr<pose_measurement::PoseMeasurement> meas( new pose_measurement::PoseMeasurement(n_zp_, n_zq_, use_fixed_covariance_, measurement_world_sensor_));
	meas->makeFromSensorReading(msg, msg->header.stamp.toSec());

	z_p_ = meas->z_p_; //store this for the init procedure
	z_q_ = meas->z_q_;

	this->manager_.msf_core_->addMeasurement(meas);
}

void PoseSensorHandler::measurementCallback(const geometry_msgs::TransformStampedConstPtr & msg)
{

        geometry_msgs::PoseWithCovarianceStampedPtr pose(new geometry_msgs::PoseWithCovarianceStamped());

        Eigen::Matrix<double, pose_measurement::nMeasurements, pose_measurement::nMeasurements> R_;

        if (!use_fixed_covariance_)  // take covariance from sensor
        {
                ROS_WARN_STREAM_THROTTLE(2,"Provided message type without covariance but set fixed_covariance=false at the same time. Discarding message.");
                return;
        }

        //fixed covariance will be set in measurement class -> makeFromSensorReadingImpl

        pose->header = msg->header;

        pose->pose.pose.position.x = msg->transform.translation.x;
        pose->pose.pose.position.y = msg->transform.translation.y;
        pose->pose.pose.position.z = msg->transform.translation.z;

        pose->pose.pose.orientation.w = msg->transform.rotation.w;
        pose->pose.pose.orientation.x = msg->transform.rotation.x;
        pose->pose.pose.orientation.y = msg->transform.rotation.y;
        pose->pose.pose.orientation.z = msg->transform.rotation.z;

        measurementCallback(pose);
}

void PoseSensorHandler::measurementCallback(const geometry_msgs::PoseStampedConstPtr & msg)
{

        geometry_msgs::PoseWithCovarianceStampedPtr pose(new geometry_msgs::PoseWithCovarianceStamped());

        Eigen::Matrix<double, pose_measurement::nMeasurements, pose_measurement::nMeasurements> R_;

        if (!use_fixed_covariance_)  // take covariance from sensor
        {
                ROS_WARN_STREAM_THROTTLE(2,"Provided message type without covariance but set fixed_covariance=false at the same time. Discarding message.");
                return;
        }

        //fixed covariance will be set in measurement class -> makeFromSensorReadingImpl

        pose->header = msg->header;

        pose->pose.pose = msg->pose;

        measurementCallback(pose);
}

