/*
 *  Created on: Nov 12, 2012
 *      Author: slynen
 */

#ifndef MEASUREMENT_HPP_
#define MEASUREMENT_HPP_

#include <msf_core/msf_fwds.hpp>
#include <Eigen/Dense>
#include <msf_core/msf_statedef.hpp>

namespace msf_core{

class MSF_Measurement{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	typedef msf_core::EKFState state_T;
	virtual ~MSF_Measurement(){}
	virtual void apply(state_T& stateWithCovariance, MSF_Core& core) = 0;
protected:
	/// main update routine called by a given sensor, will apply the measurement to the core
	template<class H_type, class Res_type, class R_type>
	bool calculateAndApplyCorrection(state_T& state, MSF_Core& core, const Eigen::MatrixBase<H_type>& H_delayed,
			const Eigen::MatrixBase<Res_type> & res_delayed, const Eigen::MatrixBase<R_type>& R_delayed);
};

//a measurement to be send to initialize parts of or the full EKF state
class MSF_InitMeasurement:public MSF_Measurement{
private:
	MSF_Measurement::state_T InitState;
public:
	template<int INDEX>
	void setStateInitValue(const typename msf_tmp::getEnumStateType<msf_core::EKFState::stateVector_T, INDEX>::value& initvalue){
		InitState.get<INDEX>() = initvalue;
		InitState.get<INDEX>().hasResetValue = true;
	}
	template<int INDEX>
	void resetStateInitValue(){
		InitState.get<INDEX>().hasResetValue = false;
	}
};

//an abstract NVI to create measurements from sensor readings
template<typename T, int MEASUREMENTSIZE>
class MSF_MeasurementCreator{
private:
	virtual void makeFromSensorReadingImpl(boost::shared_ptr<T const> reading, bool fixedCovariance) = 0;
protected:
	Eigen::Matrix<double, MEASUREMENTSIZE, MEASUREMENTSIZE> R_;
	double time_; 							///< time of this state estimate
public:
	virtual ~MSF_MeasurementCreator(){};
	void makeFromSensorReading(boost::shared_ptr<T const> reading, bool fixedCovariance = false){
		time_ = reading->header.stamp.toSec();
		makeFromSensorReadingImpl(reading, fixedCovariance);
	}
};
}

#endif /* MEASUREMENT_HPP_ */
