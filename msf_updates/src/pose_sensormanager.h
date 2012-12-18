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

#ifndef POSE_MEASUREMENTMANAGER_H
#define POSE_MEASUREMENTMANAGER_H

#include <ros/ros.h>
#include <msf_core/msf_sensormanagerROS.hpp>
#include "pose_sensorhandler.h"

class PoseSensorManager : public msf_core::MSF_SensorManagerROS
{
	friend class PoseSensorHandler;
public:
	PoseSensorManager()
	{
		PoseHandler_.reset(new PoseSensorHandler(*this));
		addHandler(PoseHandler_);
	}

private:
	boost::shared_ptr<PoseSensorHandler> PoseHandler_;

	void init(double scale)
	{
		Eigen::Matrix<double, 3, 1> p, v, b_w, b_a, g, w_m, a_m, p_ci, p_vc;
		Eigen::Quaternion<double> q, q_wv, q_ci, q_cv;
		msf_core::MSF_Core::ErrorStateCov P;

		// init values
		g << 0, 0, 9.81;	/// gravity
		b_w << 0,0,0;		/// bias gyroscopes
		b_a << 0,0,0;		/// bias accelerometer

		v << 0,0,0;			/// robot velocity (IMU centered)
		w_m << 0,0,0;		/// initial angular velocity
		a_m =g;				/// initial acceleration

		q_wv.setIdentity(); // vision-world rotation drift

		P.setZero(); // error state covariance; if zero, a default initialization in msf_core is used

		p_vc = PoseHandler_->getPositionMeasurement();
		q_cv = PoseHandler_->getAttitudeMeasurement();

		// check if we have already input from the measurement sensor
		if (p_vc.norm() == 0)
			ROS_WARN_STREAM("No measurements received yet to initialize position - using [0 0 0]");
		if ((p_vc.norm() == 1) & (q_cv.w() == 1))
			ROS_WARN_STREAM("No measurements received yet to initialize attitude - using [1 0 0 0]");

		ros::NodeHandle pnh("~");
		pnh.param("init/p_ci/x", p_ci[0], 0.0);
		pnh.param("init/p_ci/y", p_ci[1], 0.0);
		pnh.param("init/p_ci/z", p_ci[2], 0.0);

		pnh.param("init/q_ci/w", q_ci.w(), 1.0);
		pnh.param("init/q_ci/x", q_ci.x(), 0.0);
		pnh.param("init/q_ci/y", q_ci.y(), 0.0);
		pnh.param("init/q_ci/z", q_ci.z(), 0.0);
		q_ci.normalize();


		// calculate initial attitude and position based on sensor measurements
		q = (q_ci * q_cv.conjugate() * q_wv).conjugate();
		q.normalize();
		p = q_wv.conjugate().toRotationMatrix() * p_vc / scale - q.toRotationMatrix() * p_ci;

		//prepare init "measurement"
		boost::shared_ptr<msf_core::MSF_InitMeasurement> meas(new msf_core::MSF_InitMeasurement(true)); //hand over that we will also set the sensor readings

		meas->setStateInitValue<msf_core::p_>(p);
		meas->setStateInitValue<msf_core::v_>(v);
		meas->setStateInitValue<msf_core::q_>(q);
		meas->setStateInitValue<msf_core::b_w_>(b_w);
		meas->setStateInitValue<msf_core::b_a_>(b_a);
		meas->setStateInitValue<msf_core::L_>(Eigen::Matrix<double, 1, 1>::Constant(scale));
		meas->setStateInitValue<msf_core::q_wv_>(q_wv);
		meas->setStateInitValue<msf_core::q_wv_>(q_wv);
		meas->setStateInitValue<msf_core::q_ci_>(q_ci);
		meas->setStateInitValue<msf_core::p_ci_>(p_ci);

		setP(meas->get_P()); //call my set P function
		meas->get_w_m() = w_m;
		meas->get_a_m() = a_m;
		meas->time_ = ros::Time::now().toSec();

		// call initialization in core
		msf_core_->init(meas);

		ROS_INFO_STREAM("filter initialized to: \n" <<
				"position: [" << p[0] << ", " << p[1] << ", " << p[2] << "]" << std::endl <<
				"scale:" << scale << std::endl <<
				"attitude (w,x,y,z): [" << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << std::endl <<
				"p_ci: [" << p_ci[0] << ", " << p_ci[1] << ", " << p_ci[2] << std::endl <<
				"q_ci: (w,x,y,z): [" << q_ci.w() << ", " << q_ci.x() << ", " << q_ci.y() << ", " << q_ci.z() << "]");
	}
};

#endif /* POSE_MEASUREMENTMANAGER_H */
