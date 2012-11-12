/*
 * msf_measurements.hpp
 *
 *  Created on: Nov 12, 2012
 *      Author: slynen
 */

#include <msf_core/msf_core.hpp>

namespace msf_core{
MSF_SensorManager::MSF_SensorManager()
{
	msf_core_.reset(new msf_core::MSF_Core(this)); //TODO: make this a (better) design
}
}

