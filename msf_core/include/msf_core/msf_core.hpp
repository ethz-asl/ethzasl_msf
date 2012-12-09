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

#ifndef MSF_CORE_H_
#define MSF_CORE_H_


#include <Eigen/Eigen>

#include <ros/ros.h>

// message includes
#include <sensor_fusion_comm/DoubleArrayStamped.h>
#include <sensor_fusion_comm/ExtState.h>
#include <sensor_fusion_comm/ExtEkf.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>

#include <msf_core/msf_sortedContainer.hpp>
#include <vector>
#include <msf_core/msf_state.hpp>

//good old days...
//#define N_STATE_BUFFER 256	///< size of unsigned char, do not change!

namespace msf_core{

enum{
	HLI_EKF_STATE_SIZE = 16 ///< number of states exchanged with external propagation. Here: p,v,q,bw,bw=16
};

class MSF_SensorManager;

class MSF_Core
{
	bool initialized_;
	bool predictionMade_;
public:
	friend class MSF_MeasurementBase;
	enum{
		nErrorStatesAtCompileTime = msf_core::EKFState::nErrorStatesAtCompileTime,  ///< error state
		nStatesAtCompileTime = msf_core::EKFState::nStatesAtCompileTime ///< complete state
	};
	typedef Eigen::Matrix<double, nErrorStatesAtCompileTime, 1> ErrorState;
	typedef Eigen::Matrix<double, nErrorStatesAtCompileTime, nErrorStatesAtCompileTime> ErrorStateCov;

	typedef msf_core::SortedContainer<msf_core::EKFState> stateBufferT;
	typedef msf_core::SortedContainer<msf_core::MSF_MeasurementBase, msf_core::MSF_InvalidMeasurement> measurementBufferT;

	void addMeasurement(boost::shared_ptr<MSF_MeasurementBase> measurement);

	void init(boost::shared_ptr<MSF_MeasurementBase> measurement);

	void initExternalPropagation(boost::shared_ptr<EKFState> state);

	boost::shared_ptr<EKFState> getClosestState(double tstamp, double delay = 0.00);

	//delete very old states and measurements from the buffers to free memory
	void CleanUpBuffers(){
		double timeold = 60; //1 min
		StateBuffer_.clearOlderThan(timeold);
		MeasurementBuffer_.clearOlderThan(timeold);
	}

	void setPCore(Eigen::Matrix<double, msf_core::EKFState::nErrorStatesAtCompileTime, msf_core::EKFState::nErrorStatesAtCompileTime>& P){
		enum{
			coreErrorStates = 15 // we might want to calculate this, but on the other hand the values for the matrix later on are anyway hardcoded
		};
		P.setIdentity();
		P *= 0.0001; //set diagonal small covariance for all states

		//now set the core state covariance to the simulated values
		Eigen::Matrix<double, coreErrorStates, coreErrorStates> P_core;
		P_core<<  0.0166, 0.0122,-0.0015, 0.0211, 0.0074, 0.0000, 0.0012,-0.0012, 0.0001,-0.0000, 0.0000,-0.0000,-0.0003,-0.0002,-0.0000,
				0.0129, 0.0508,-0.0020, 0.0179, 0.0432, 0.0006, 0.0020, 0.0004,-0.0002,-0.0000, 0.0000, 0.0000, 0.0003,-0.0002, 0.0000,
				-0.0013,-0.0009, 0.0142,-0.0027, 0.0057, 0.0079, 0.0007, 0.0007, 0.0000,-0.0000,-0.0000, 0.0000,-0.0001,-0.0004,-0.0001,
				0.0210, 0.0162,-0.0026, 0.0437, 0.0083,-0.0017, 0.0016,-0.0021,-0.0014,-0.0000, 0.0000, 0.0000, 0.0003,-0.0001, 0.0000,
				0.0093, 0.0461, 0.0036, 0.0153, 0.0650,-0.0016, 0.0025, 0.0013,-0.0000,-0.0000, 0.0000, 0.0000, 0.0003, 0.0002, 0.0000,
				-0.0000, 0.0005, 0.0080,-0.0019,-0.0021, 0.0130, 0.0001, 0.0001, 0.0000,-0.0000, 0.0000,-0.0000,-0.0003, 0.0001,-0.0001,
				0.0012, 0.0024, 0.0006, 0.0017, 0.0037, 0.0001, 0.0005, 0.0000, 0.0001,-0.0000, 0.0000,-0.0000,-0.0000,-0.0001,-0.0000,
				-0.0011, 0.0008, 0.0007,-0.0023, 0.0019, 0.0001, 0.0000, 0.0005,-0.0001,-0.0000,-0.0000, 0.0000, 0.0001,-0.0001,-0.0000,
				0.0001,-0.0002,-0.0000,-0.0014, 0.0001, 0.0000, 0.0000,-0.0001, 0.0006,-0.0000,-0.0000,-0.0000, 0.0000, 0.0000,-0.0000,
				-0.0000,-0.0000,-0.0000,-0.0000,-0.0000,-0.0000,-0.0000,-0.0000,-0.0000, 0.0000,-0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
				0.0000, 0.0000,-0.0000, 0.0000, 0.0000, 0.0000, 0.0000,-0.0000,-0.0000,-0.0000, 0.0000,-0.0000, 0.0000, 0.0000, 0.0000,
				-0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000,-0.0000, 0.0000,-0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000,-0.0000,
				-0.0003, 0.0003,-0.0001, 0.0003, 0.0003,-0.0003,-0.0000, 0.0001, 0.0000, 0.0000, 0.0000, 0.0000, 0.0010, 0.0000, 0.0000,
				-0.0002,-0.0002,-0.0004,-0.0001, 0.0003, 0.0001,-0.0001,-0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0010, 0.0000,
				-0.0000, 0.0000,-0.0001, 0.0000, 0.0000,-0.0001,-0.0000,-0.0000,-0.0000, 0.0000, 0.0000,-0.0000, 0.0000, 0.0000, 0.0001;
		P.block<coreErrorStates, coreErrorStates>(0,0) = P_core;
	}


	MSF_Core(MSF_SensorManager* usercalc);
	~MSF_Core();


private:
	Eigen::Matrix<double, nErrorStatesAtCompileTime, nErrorStatesAtCompileTime> Fd_; ///< discrete state propagation matrix
	Eigen::Matrix<double, nErrorStatesAtCompileTime, nErrorStatesAtCompileTime> Qd_; ///< discrete propagation noise matrix

	/// state variables
	stateBufferT StateBuffer_; ///<EKF buffer containing pretty much all info needed at time t
	measurementBufferT MeasurementBuffer_; ///<EKF Measurement

	double time_P_propagated;

	Eigen::Matrix<double, 3, 1> g_; ///< gravity vector

	Eigen::Matrix<double, 3, 3> R_IW_; ///< Rot IMU->World
	Eigen::Matrix<double, 3, 3> R_CI_; ///< Rot Camera->IMU
	Eigen::Matrix<double, 3, 3> R_WV_; ///< Rot World->Vision

	/// vision-world drift watch dog to determine fuzzy tracking

	//get the index of the best state having no temporal drift at compile time
	enum{
		indexOfStateWithoutTemporalDrift = msf_tmp::IndexOfBestNonTemporalDriftingState<msf_core::fullState_T>::value
	};
	typedef typename msf_tmp::getEnumStateType<msf_core::fullState_T, indexOfStateWithoutTemporalDrift>::value nonDriftingStateType; //returns void type for invalid types

	const static int qbuffRowsAtCompiletime = msf_tmp::StateLengthForType<const msf_tmp::StripConstReference<nonDriftingStateType>::result_t&>::value;

	const static int nBuff_ = 30; ///< buffer size for median q_vw
	int nontemporaldrifting_inittimer_;
	Eigen::Matrix<double, nBuff_, qbuffRowsAtCompiletime> qbuff_; //if there is no non temporal drifting state this matrix will have zero rows, to make use of it illegal


	/// enables internal state predictions for log replay
	/**
	 * used to determine if internal states get overwritten by the external
	 * state prediction (online) or internal state prediction is performed
	 * for log replay, when the external prediction is not available.
	 */
	bool data_playback_;

	MSF_SensorManager* usercalc_; //a function which provides methods for customization

	enum
	{
		NO_UP, GOOD_UP, FUZZY_UP
	};

	ros::Publisher pubState_; ///< publishes all states of the filter
	sensor_fusion_comm::DoubleArrayStamped msgState_;

	ros::Publisher pubPose_; ///< publishes 6DoF pose output
	geometry_msgs::PoseWithCovarianceStamped msgPose_;

	ros::Publisher pubPoseCrtl_; ///< publishes 6DoF pose including velocity output
	sensor_fusion_comm::ExtState msgPoseCtrl_;

	ros::Publisher pubCorrect_; ///< publishes corrections for external state propagation
	sensor_fusion_comm::ExtEkf msgCorrect_;

	ros::Subscriber subState_; ///< subscriber to external state propagation
	ros::Subscriber subImu_; ///< subscriber to IMU readings

	sensor_fusion_comm::ExtEkf hl_state_buf_; ///< buffer to store external propagation data

	/// propagates the state with given dt
	void propagateState(boost::shared_ptr<EKFState>& state_old, boost::shared_ptr<EKFState>& state_new);

	/// propagets the error state covariance
	void predictProcessCovariance(boost::shared_ptr<EKFState>& state_old, boost::shared_ptr<EKFState>& state_new);

	/// applies the correction
	bool applyCorrection(boost::shared_ptr<EKFState>& delaystate, ErrorState & correction, double fuzzythres = 0.1);

	/// propagate covariance to a given state in time
	void propPToState(boost::shared_ptr<EKFState>& state);

	/// internal state propagation
	/**
	 * This function gets called on incoming imu messages an then performs
	 * the state prediction internally. Only use this OR stateCallback by
	 * remapping the topics accordingly.
	 * \sa{stateCallback}
	 */
	void imuCallback(const sensor_msgs::ImuConstPtr & msg);

	/// external state propagation
	/**
	 * This function gets called when state prediction is performed externally,
	 * e.g. by asctec_mav_framework. Msg has to be the latest predicted state.
	 * Only use this OR imuCallback by remapping the topics accordingly.
	 * \sa{imuCallback}
	 */
	void stateCallback(const sensor_fusion_comm::ExtEkfConstPtr & msg);


};

};// end namespace

#include <msf_core/implementation/msf_core.hpp>

#endif /* MSF_CORE_H_ */
