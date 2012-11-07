/*
 * msf_core.hpp
 *
 *  Created on: Nov 6, 2012
 *      Author: slynen
 */

#ifndef MSF_CORE_HPP_
#define MSF_CORE_HPP_

#include <msf_core/msf_tmp.hpp>
#include <msf_core/msf_statedef.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <msf_core/eigen_conversions.h>
#include <sensor_fusion_comm/ExtState.h>
#include <sensor_fusion_comm/DoubleArrayStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

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
	value_t state_;
};

template<typename stateVector_T>
struct EKFState_T{
	typedef stateVector_T state_T;
	enum{
		nstatevars_ = boost::fusion::result_of::size<state_T>::type::value, //n all states
		nerrorstates_ = msf_tmp::CountStates<state_T, msf_tmp::CorrectionStateLengthForType>::value, //n correction states
		nstates_ = msf_tmp::CountStates<state_T, msf_tmp::StateLengthForType>::value //n correction states
	};
	stateVector_T statevars_; ///< the actual state variables

	// system inputs
	Eigen::Matrix<double,3,1> w_m_;         ///< angular velocity from IMU
	Eigen::Matrix<double,3,1> a_m_;         ///< acceleration from IMU

	Eigen::Quaternion<double> q_int_;       ///< this is the integrated ang. vel. no corrections applied, to use for delta rot in external algos...

	double time_; 				///< time of this state estimate
	Eigen::Matrix<double, nerrorstates_, nerrorstates_> P_;///< error state covariance

	//apply the correction vector to all state vars
	void correct(const Eigen::Matrix<double, nerrorstates_, 1>& correction) {
		boost::fusion::for_each(
				statevars_,
				msf_tmp::correctState<const Eigen::Matrix<double, nerrorstates_, 1>, state_T >(correction)
		);
	}

	//returns the state at position INDEX in the state list
	template<int INDEX>
	typename boost::fusion::result_of::at_c<state_T, INDEX >::type
	get(){
		return boost::fusion::at<boost::mpl::int_<INDEX> >(statevars_);
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

		//set scale to 1
		get<msf_core::L_>()(0) = 1.0;

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

		typedef typename msf_tmp::getEnumStateType<state_T, msf_core::p_>::value p_type;
		typedef typename msf_tmp::getEnumStateType<state_T, msf_core::q_>::value q_type;

		static const int idxstartcorr_p = msf_tmp::getStartIndex<state_T, p_type, msf_tmp::CorrectionStateLengthForType>::value;
		static const int idxstartstate_p = msf_tmp::getStartIndex<state_T, p_type, msf_tmp::StateLengthForType>::value;
		static const int idxstartcorr_q = msf_tmp::getStartIndex<state_T, q_type, msf_tmp::CorrectionStateLengthForType>::value;
		static const int idxstartstate_q = msf_tmp::getStartIndex<state_T, q_type, msf_tmp::StateLengthForType>::value;

#pragma unroll(3)
		for (int i = 0; i < 9; i++)
			cov[i / 3 * 6 + i % 3] = P_((i + idxstartcorr_p) / 3 * nerrorstates_ + (i + idxstartcorr_p) % 3);


		cov[0] = P_(0);
		cov[1] = P_(1);
		cov[2] = P_(2);
		cov[6] = P_(25);
		cov[7] = P_(26);
		cov[8] = P_(27);
		cov[12] = P_(50);
		cov[13] = P_(51);
		cov[14] = P_(52);

		//TODO fix this
#pragma unroll(3)
		for (int i = 0; i < 9; i++)
			cov[i / 3 * 6 + (i % 3 + 3)] = P_(i / 3 * nerrorstates_ + (i % 3 + 6));

		cov[3] = P_(6);
		cov[4] = P_(7);
		cov[5] = P_(8);
		cov[9] = P_(31);
		cov[10] = P_(32);
		cov[11] = P_(33);
		cov[15] = P_(56);
		cov[16] = P_(57);
		cov[17] = P_(58);

		//		  for (int i = 0; i < 9; i++)
		//		    cov[(i / 3 + 3) * 6 + i % 3] = P_((i / 3 + 6) * N_STATE + i % 3);
		//
		//		  for (int i = 0; i < 9; i++)
		//		    cov[(i / 3 + 3) * 6 + (i % 3 + 3)] = P_((i / 3 + 6) * N_STATE + (i % 3 + 6));
	}

	/// assembles a PoseWithCovarianceStamped message from the state
	/** it does not set the header */
	void toPoseMsg(geometry_msgs::PoseWithCovarianceStamped & pose){
		eigen_conversions::vector3dToPoint(get<msf_core::p_>(), pose.pose.pose.position);
		eigen_conversions::quaternionToMsg(get<msf_core::q_>(), pose.pose.pose.orientation);
		getPoseCovariance(pose.pose.covariance);
	}

	/// assembles an ExtState message from the state
	/** it does not set the header */
	void toExtStateMsg(sensor_fusion_comm::ExtState & state){
		eigen_conversions::vector3dToPoint(get<msf_core::p_>(), state.pose.position);
		eigen_conversions::quaternionToMsg(get<msf_core::q_>(), state.pose.orientation);
		eigen_conversions::vector3dToPoint(get<msf_core::v_>(), state.velocity);
	}

	/// assembles a DoubleArrayStamped message from the state
	/** it does not set the header */
	void toStateMsg(sensor_fusion_comm::DoubleArrayStamped & state){
		state.data.resize(nstates_); //make sure this is correctly sized
		boost::fusion::for_each(
				statevars_,
				msf_tmp::StatetoDoubleArray<std::vector<double>, state_T >(state.data)
		);
	}

};

#endif /* MSF_CORE_HPP_ */
