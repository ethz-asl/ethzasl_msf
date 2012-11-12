/*

Copyright (c) 2010, Stephan Weiss, ASL, ETH Zurich, Switzerland
You can contact the author at <stephan dot weiss at ieee dot org>

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

#ifndef POSITION_MEASUREMENTS_H
#define POSITION_MEASUREMENTS_H

#include <ros/ros.h>
#include <msf_core/msf_sensormanagerROS.hpp>
#include "position_sensor.h"

class PositionMeasurements: public msf_core::MSF_SensorManagerROS
{
public:
	PositionMeasurements()
	{
		addHandler(new PositionSensorHandler(this));

	    ros::NodeHandle pnh("~");
	    pnh.param("init/p_ci/x", p_ci_[0], 0.0);
	    pnh.param("init/p_ci/y", p_ci_[1], 0.0);
	    pnh.param("init/p_ci/z", p_ci_[2], 0.0);

	    pnh.param("init/q_ci/w", q_ci_.w(), 1.0);
	    pnh.param("init/q_ci/x", q_ci_.x(), 0.0);
	    pnh.param("init/q_ci/y", q_ci_.y(), 0.0);
	    pnh.param("init/q_ci/z", q_ci_.z(), 0.0);
	    q_ci_.normalize();
	}

private:

	  Eigen::Matrix<double, 3, 1> p_ci_; ///< initial distance camera-IMU
	  Eigen::Quaternion<double> q_ci_; ///< initial rotation camera-IMU

	void init(double scale)
	{
	    Eigen::Matrix<double, 3, 1> p, v, b_w, b_a, g, w_m, a_m;
	    Eigen::Quaternion<double> q, q_wv;
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

		// check if we have already input from the measurement sensor
		if(p_vc_.norm()==0)
			ROS_WARN_STREAM("No measurements received yet to initialize position - using [0 0 0]");
		if((q_cv_.norm()==1) & (q_cv_.w()==1))
			ROS_WARN_STREAM("No measurements received yet to initialize attitude - using [1 0 0 0]");

		// calculate initial attitude and position based on sensor measurements
		q = (q_ci_ * q_cv_.conjugate() * q_wv).conjugate();
		q.normalize();
		p = q_wv.conjugate().toRotationMatrix()*p_vc_/scale - q.toRotationMatrix()*p_ci_;


		// call initialization in core
		msf_core_.initialize(p,v,q,b_w,b_a,P,w_m,a_m,g);

	    ROS_INFO_STREAM("filter initialized to: \n" <<
	        "position: [" << p[0] << ", " << p[1] << ", " << p[2] << "]" << std::endl <<
	        "scale:" << scale << std::endl <<
	        "attitude (w,x,y,z): [" << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << std::endl <<
	        "p_ci: [" << p_ci_[0] << ", " << p_ci_[1] << ", " << p_ci_[2] << std::endl <<
	        "q_ci: (w,x,y,z): [" << q_ci_.w() << ", " << q_ci_.x() << ", " << q_ci_.y() << ", " << q_ci_.z() << "]");
	}
};

#endif /* POSITION_MEASUREMENTS_H */
