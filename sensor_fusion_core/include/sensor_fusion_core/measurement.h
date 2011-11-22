/*
 * measurement.h
 *
 *  Created on: Sep 30, 2010
 *      Author: sweiss
 */

#ifndef MEASUREMENT_H
#define MEASUREMENT_H

#include <sensor_fusion_core/Sensor_Fusion_Core.h>
#include <map>

class MeasurementHandler;

class Measurements
{
protected:
//	typedef std::map<int, MeasurementHandler*> Handlers;
//	Handlers handlers;

	typedef std::vector<MeasurementHandler*> Handlers;
	Handlers handlers;

	ReconfigureServer *reconfServer_;

	void Config(sensor_fusion_core::Sensor_Fusion_CoreConfig &config, uint32_t level);
	virtual	void init(double scale) = 0;

public:
	Sensor_Fusion_Core poseFilter_;

	void addHandler(int id, MeasurementHandler* handler)
	{

		handlers.push_back(handler);
//		handlers.push_back(std::pair<int, MeasurementHandler*>(id, handler));
//		handlers[id] = handler;
	}
	Measurements();
	virtual ~Measurements();
};


class MeasurementHandler
{
protected:
	Measurements* measurements;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	MeasurementHandler(Measurements* meas):measurements(meas){}
//	virtual Eigen::VectorXd init(){return Eigen::Matrix<double, 1, 1>::Zero();}

	virtual ~MeasurementHandler() {}
};





#endif /* MEASUREMENT_H */
