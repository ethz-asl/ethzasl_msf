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

class MSF_MeasurementBase{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	typedef msf_core::EKFState state_T;
	virtual ~MSF_MeasurementBase(){}
	virtual void apply(boost::shared_ptr<EKFState> stateWithCovariance, MSF_Core& core) = 0;
	double time_;
protected:
	/// main update routine called by a given sensor, will apply the measurement to the core
	template<class H_type, class Res_type, class R_type>
	void calculateAndApplyCorrection(boost::shared_ptr<EKFState> state, MSF_Core& core, const Eigen::MatrixBase<H_type>& H_delayed,
			const Eigen::MatrixBase<Res_type> & res_delayed, const Eigen::MatrixBase<R_type>& R_delayed);
};


class MSF_InvalidMeasurement:public MSF_MeasurementBase{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	virtual void apply(boost::shared_ptr<EKFState> stateWithCovariance, MSF_Core& core){
		ROS_ERROR_STREAM("Called apply() on an MSF_InvalidMeasurement object. This should never happen.");
	}
};

//an abstract NVI to create measurements from sensor readings
template<typename T, int MEASUREMENTSIZE>
class MSF_Measurement: public MSF_MeasurementBase{
private:
	virtual void makeFromSensorReadingImpl(boost::shared_ptr<T const> reading, bool fixedCovariance) = 0;
protected:
	Eigen::Matrix<double, MEASUREMENTSIZE, MEASUREMENTSIZE> R_;
public:
	virtual ~MSF_Measurement(){};
	void makeFromSensorReading(boost::shared_ptr<T const> reading, bool fixedCovariance = false){
		time_ = reading->header.stamp.toSec();
		makeFromSensorReadingImpl(reading, fixedCovariance);
	}
	//apply is implemented by respective sensor measurement types
};

//a measurement to be send to initialize parts of or the full EKF state
class MSF_InitMeasurement:public MSF_MeasurementBase{
private:
	MSF_MeasurementBase::state_T InitState;
	bool ContainsInitialSensorReadings_;
	typedef MSF_MeasurementBase::state_T::stateVector_T stateVector_T;
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	MSF_InitMeasurement(bool ContainsInitialSensorReadings){
		ContainsInitialSensorReadings_ = ContainsInitialSensorReadings;
		time_ = ros::Time::now().toSec();
	}
	virtual ~MSF_InitMeasurement(){};

	MSF_MeasurementBase::state_T::P_type& get_P(){
		return InitState.P_;
	}
	Eigen::Matrix<double, 3, 1>& get_w_m(){
		return InitState.w_m_;
	}
	Eigen::Matrix<double, 3, 1>& get_a_m(){
		return InitState.a_m_;
	}

	template<int INDEX, typename T>
	void setStateInitValue(const T& initvalue){
		InitState.getStateVar<INDEX>().state_ = initvalue;
		InitState.getStateVar<INDEX>().hasResetValue = true;
	}
	template<int INDEX>
	void resetStateInitValue(){
		InitState.get<INDEX>().hasResetValue = false;
	}
	virtual void apply(boost::shared_ptr<EKFState> stateWithCovariance, MSF_Core& core);
};


template<typename stateSequence_T>
class sortMeasurements
{
public:
	bool operator() (const MSF_MeasurementBase& lhs, const MSF_MeasurementBase&rhs) const
	{
		return (lhs.time_<rhs.time_);
	}
};

}

#include <msf_core/implementation/msf_measurement.hpp>

#endif /* MEASUREMENT_HPP_ */
