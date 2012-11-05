/*
 * measurement.cpp
 *
 *  Created on: Sep 30, 2010
 *      Author: sweiss
 */

#include <sensor_fusion_core/measurement.h>

Measurements::Measurements()
{
	poseFilter_.registerCallback(&Measurements::Config,this);
}

Measurements::~Measurements()
{
	for (Handlers::iterator it(handlers.begin()); it != handlers.end(); ++it)
		delete *it;
//		delete it->second;

	delete reconfServer_;
	return;
}


void Measurements::Config(sensor_fusion_core::Sensor_Fusion_CoreConfig& config, uint32_t level){
	if(level &sensor_fusion_core::Sensor_Fusion_Core_INIT_FILTER){
		init(config.scale_init);
		config.init_filter = false;
	}
}




