/*
 * vismaggps_measurements.h
 *
 *  Created on: Sep 30, 2010
 *      Author: sweiss
 */

#ifndef VISMAGGPS_MEASUREMENTS_H_
#define VISMAGGPS_MEASUREMENTS_H_

#include <sensor_fusion_core/measurement.h>
#include "vismaggps.h"

namespace Sensors
{
  enum Sensor
  {
     VISION=1,
     MAG=2,
     GPS=3,
     VISMAGGPS=4
  };
}
typedef Sensors::Sensor Sensor;

class VisMagGPSMeasurements: public Measurements
{
public:
	// measurements
	Eigen::Quaternion<double> q_cv_;	// attitude camera -> vision frame
	Eigen::Matrix<double, 3, 1> p_vc_;	// position camera -> vision frame
	Eigen::Matrix<double, 3, 1> p_m_;	// mag direction vector in mag frame
	Eigen::Matrix<double, 3, 1> p_wg_;	// position world -> GPS sensor
	Eigen::Matrix<double, 2, 1> v_wg_;	// velocity world -> GPS sensor
	VisMagGPSMeasurements()
	{
		addHandler(Sensors::VISMAGGPS, new VisMagGPSHandler(this));
		poseFilter_.registerCallback(&VisMagGPSMeasurements::Config, this);
	};

	void init(double scale)
	{
		Eigen::Matrix<double, 3, 1> p,v,b_w,b_a,p_ic,g,w_m,a_m, p_ig, p_vw;
		Eigen::Quaternion<double> q, q_ci,q_wv,q_mi;
		Eigen::Matrix<double,nState_,nState_> P;

		g << 0, 0, 9.81;
		b_w << 0,0,0;
		b_a << 0,0,0;

//		Eigen::VectorXd data = handlers[Sensors::VISION]->init();

		v << 0,0,0;
		w_m << 0,0,0;
		a_m =g;	// zero vel initialization....

		p_ic = Eigen::Matrix<double,3,1>(0, 0, 0);
		q_ci=Eigen::Quaternion<double>(0, 1, 0, 0);
		q_ci.normalize();
		q_wv=Eigen::Quaternion<double>(1, 0, 0, 0);
		q_wv.normalize();
		q_mi=Eigen::Quaternion<double>(1, 0, 0, 0);
		q_mi.normalize();
		p_ig = Eigen::Matrix<double,3,1>(0, 0, 0);
		p_vw = Eigen::Matrix<double,3,1>(0, 0, 0);
		double alpha=0;
		double beta=0;


		P = Eigen::Matrix<double,nState_,nState_>::Constant(0);

		if(p_vc_.norm()==0)
			ROS_WARN_STREAM("No measurements received yet to initialize position - using [0 0 0]");
		if((q_cv_.norm()==1) & (q_cv_.w()==1))
			ROS_WARN_STREAM("No measurements received yet to initialize attitude - using [1 0 0 0]");

		q = ( q_ci * q_cv_ *q_wv).conjugate();
		p = q_wv.conjugate().toRotationMatrix()*p_vc_/scale - q.toRotationMatrix()*p_ic ;

		poseFilter_.initialize(p,v,q,b_w,b_a,scale,q_wv,P,w_m,a_m,g,q_ci,p_ic,q_mi,p_ig,p_vw,alpha,beta);

		ROS_INFO_STREAM("filter initialized to: \n" <<
						"position: [" << p[0] << ", " << p[1] << ", " << p[2] << "]" <<
						"attitude (w,x,y,z): [" << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << "]");
		};


	void Config(sensor_fusion_core::Sensor_Fusion_CoreConfig& config, uint32_t level){

		if(level & sensor_fusion_core::Sensor_Fusion_Core_SET_HEIGHT){
			if(p_vc_.norm()==0)
			{
				ROS_WARN_STREAM("No measurements received yet to initialize position - using scale factor " << config.scale_init << " for init");
				init(config.scale_init);
			}
			else
				init(p_vc_[2]/config.set_height);
			config.set_height = false;
		}
	}
};

#endif /* VISMAGGPS_MEASUREMENTS_H_ */

