/*
 *  Created on: Nov 7, 2012
 *      Author: slynen
 */



#ifndef MSF_STATE_HPP_
#define MSF_STATE_HPP_

#include <msf_core/msf_types.hpp>
#include <msf_core/msf_tmp.hpp>
#include <msf_core/msf_statedef.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <msf_core/eigen_conversions.h>
#include <sensor_fusion_comm/ExtState.h>
#include <sensor_fusion_comm/DoubleArrayStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace msf_core{

//a state variable with a name as specified in the state name enum
template<typename type_T, int name_T>
struct StateVar_T{
	typedef type_T value_t;
	typedef const StateVar_T<type_T, name_T>& constRef_T;
	typedef const StateVar_T<type_T, name_T>* constPtr_T;
	typedef StateVar_T<type_T, name_T>& Ref_T;
	typedef StateVar_T<type_T, name_T>* Ptr_T;
	enum{
		name_ = name_T,
		sizeInCorrection_ = msf_tmp::CorrectionStateLengthForType<const StateVar_T<type_T, name_T>&>::value,
		sizeInState_ = msf_tmp::StateLengthForType<const StateVar_T<type_T, name_T>&>::value
	};
	//	typedef Eigen::Matrix<double, sizeInCorrection_, sizeInCorrection_> Q_T;
	//	Q_T Q;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	value_t state_;
};


template<typename stateSequence_T>
struct GenericState_T{
	typedef stateSequence_T stateVector_T;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	enum{
		nStateVarsAtCompileTime = boost::fusion::result_of::size<stateVector_T>::type::value, //n state vars
		nErrorStatesAtCompileTime = msf_tmp::CountStates<stateVector_T, msf_tmp::CorrectionStateLengthForType>::value, //n correction states
		nStatesAtCompileTime = msf_tmp::CountStates<stateVector_T, msf_tmp::StateLengthForType>::value //n total states
	};
	stateVector_T statevars_; ///< the actual state variables

	// system inputs
	Eigen::Matrix<double,3,1> w_m_;         ///< angular velocity from IMU
	Eigen::Matrix<double,3,1> a_m_;         ///< acceleration from IMU

	Eigen::Quaternion<double> q_int_;       ///< this is the integrated ang. vel. no corrections applied, to use for delta rot in external algos...

	double time_; 				///< time of this state estimate
	Eigen::Matrix<double, nErrorStatesAtCompileTime, nErrorStatesAtCompileTime> P_;///< error state covariance


	//apply the correction vector to all state vars
	inline void correct(const Eigen::Matrix<double, nErrorStatesAtCompileTime, 1>& correction) {
		boost::fusion::for_each(
				statevars_,
				msf_tmp::correctState<const Eigen::Matrix<double, nErrorStatesAtCompileTime, 1>, stateVector_T >(correction)
		);
	}

	//returns the state at position INDEX in the state list
	template<int INDEX>
	inline typename boost::fusion::result_of::at_c<stateVector_T, INDEX >::type&
	get(){
		return boost::fusion::at<boost::mpl::int_<INDEX> >(statevars_);
	}

	//returns the state at position INDEX in the state list
	template<int INDEX>
	inline typename boost::fusion::result_of::at_c<stateVector_T, INDEX >::type::Q_T
	getQBlock(){
		return boost::fusion::at<boost::mpl::int_<INDEX> >(statevars_).Q;
	}

	/// resets the state
	/**
	 * 3D vectors: 0; quaternion: unit quaternion; scale: 1; time:0; Error covariance: zeros
	 */
	void reset(){
		// reset all states
		boost::fusion::for_each(
				statevars_,
				msf_tmp::resetState()
		);
		//reset system inputs
		w_m_.setZero();
		a_m_.setZero();

		q_int_.setIdentity();

		P_.setZero();
		time_ = 0;
	}



	/// writes the covariance corresponding to position and attitude to cov
	void getPoseCovariance(geometry_msgs::PoseWithCovariance::_covariance_type & cov){
		BOOST_STATIC_ASSERT(geometry_msgs::PoseWithCovariance::_covariance_type::static_size == 36);

		typedef typename msf_tmp::getEnumStateType<stateVector_T, msf_core::p_>::value p_type;
		typedef typename msf_tmp::getEnumStateType<stateVector_T, msf_core::q_>::value q_type;

		//get indices of position and attitude in the covariance matrix
		static const int idxstartcorr_p = msf_tmp::getStartIndex<stateVector_T, p_type, msf_tmp::CorrectionStateLengthForType>::value;
		static const int idxstartcorr_q = msf_tmp::getStartIndex<stateVector_T, q_type, msf_tmp::CorrectionStateLengthForType>::value;

		//TODO: remove the following two lines after initial debug to allow changes in state ordering
		BOOST_STATIC_ASSERT(idxstartcorr_p==0);
		BOOST_STATIC_ASSERT(idxstartcorr_q==6);

		/*        |  cov_p_p  |  cov_p_q  |
		 *        |           |           |
		 * cov =  |-----------|-----------|
		 *        |           |           |
		 *        |  cov_q_p  |  cov_q_q  |
		 */

		for (int i = 0; i < 9; i++)
			cov[i / 3 * 6 + i % 3] = P_((i / 3 + idxstartcorr_p) * nErrorStatesAtCompileTime + i % 3);

		for (int i = 0; i < 9; i++)
			cov[i / 3 * 6 + (i % 3 + 3)] = P_((i / 3 + idxstartcorr_p) * nErrorStatesAtCompileTime + (i % 3 + 6));

		for (int i = 0; i < 9; i++)
			cov[(i / 3 + 3) * 6 + i % 3] = P_((i / 3 + idxstartcorr_q) * nErrorStatesAtCompileTime + i % 3);

		for (int i = 0; i < 9; i++)
			cov[(i / 3 + 3) * 6 + (i % 3 + 3)] = P_((i / 3 + idxstartcorr_q) * nErrorStatesAtCompileTime + (i % 3 + 6));
	}

	/// assembles a PoseWithCovarianceStamped message from the state
	/** it does not set the header */
	void toPoseMsg(geometry_msgs::PoseWithCovarianceStamped & pose){
		eigen_conversions::vector3dToPoint(get<msf_core::p_>().state_, pose.pose.pose.position);
		eigen_conversions::quaternionToMsg(get<msf_core::q_>().state_, pose.pose.pose.orientation);
		getPoseCovariance(pose.pose.covariance);
	}

	/// assembles an ExtState message from the state
	/** it does not set the header */
	void toExtStateMsg(sensor_fusion_comm::ExtState & state){
		eigen_conversions::vector3dToPoint(get<msf_core::p_>().state_, state.pose.position);
		eigen_conversions::quaternionToMsg(get<msf_core::q_>().state_, state.pose.orientation);
		eigen_conversions::vector3dToPoint(get<msf_core::v_>().state_, state.velocity);
	}

	/// assembles a DoubleArrayStamped message from the state
	/** it does not set the header */
	void toStateMsg(sensor_fusion_comm::DoubleArrayStamped & state){
		state.data.resize(nStatesAtCompileTime); //make sure this is correctly sized
		boost::fusion::for_each(
				statevars_,
				msf_tmp::StatetoDoubleArray<std::vector<double>, stateVector_T >(state.data)
		);
	}
};
}
#endif /* MSF_STATE_HPP_ */
