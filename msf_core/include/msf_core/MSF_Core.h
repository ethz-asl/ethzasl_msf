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

#ifndef SSF_CORE_H_
#define SSF_CORE_H_


#include <Eigen/Eigen>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <msf_core/MSF_CoreConfig.h>

// message includes
#include <sensor_fusion_comm/DoubleArrayStamped.h>
#include <sensor_fusion_comm/ExtState.h>
#include <sensor_fusion_comm/ExtEkf.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>

#include <vector>
#include <msf_core/state.h>

#define N_STATE_BUFFER 256	///< size of unsigned char, do not change!
#define HLI_EKF_STATE_SIZE 16 	///< number of states exchanged with external propagation. Here: p,v,q,bw,bw=16

namespace msf_core{

typedef dynamic_reconfigure::Server<msf_core::MSF_CoreConfig> ReconfigureServer;

class MSF_Core
{

public:
  typedef Eigen::Matrix<double, N_STATE, 1> ErrorState;
  typedef Eigen::Matrix<double, N_STATE, N_STATE> ErrorStateCov;

  /// big init routine
  void initialize(const Eigen::Matrix<double, 3, 1> & p, const Eigen::Matrix<double, 3, 1> & v,
                  const Eigen::Quaternion<double> & q, const Eigen::Matrix<double, 3, 1> & b_w,
                  const Eigen::Matrix<double, 3, 1> & b_a, const double & L, const Eigen::Quaternion<double> & q_wv,
                  const Eigen::Matrix<double, N_STATE, N_STATE> & P, const Eigen::Matrix<double, 3, 1> & w_m,
                  const Eigen::Matrix<double, 3, 1> & a_m, const Eigen::Matrix<double, 3, 1> & g,
                  const Eigen::Quaternion<double> & q_ci, const Eigen::Matrix<double, 3, 1> & p_ci);

  /// retreive all state information at time t. Used to build H, residual and noise matrix by update sensors
  unsigned char getClosestState(State* timestate, ros::Time tstamp, double delay = 0.00);

  /// get all state information at a given index in the ringbuffer
  bool getStateAtIdx(State* timestate, unsigned char idx);

  MSF_Core();
  ~MSF_Core();

private:
  const static int nFullState_ = 28; ///< complete state
  const static int nBuff_ = 30; ///< buffer size for median q_vw
  const static int nMaxCorr_ = 50; ///< number of IMU measurements buffered for time correction actions
  const static int QualityThres_ = 1e3;

  Eigen::Matrix<double, N_STATE, N_STATE> Fd_; ///< discrete state propagation matrix
  Eigen::Matrix<double, N_STATE, N_STATE> Qd_; ///< discrete propagation noise matrix

  /// state variables
  State StateBuffer_[N_STATE_BUFFER]; ///< EKF ringbuffer containing pretty much all info needed at time t
  unsigned char idx_state_; ///< pointer to state buffer at most recent state
  unsigned char idx_P_; ///< pointer to state buffer at P latest propagated
  unsigned char idx_time_; ///< pointer to state buffer at a specific time

  Eigen::Matrix<double, 3, 1> g_; ///< gravity vector

  /// vision-world drift watch dog to determine fuzzy tracking
  int qvw_inittimer_;
  Eigen::Matrix<double, nBuff_, 4> qbuff_;

  /// correction from EKF update
  Eigen::Matrix<double, N_STATE, 1> correction_;

  /// dynamic reconfigure config
  msf_core::MSF_CoreConfig config_;

  Eigen::Matrix<double, 3, 3> R_IW_; ///< Rot IMU->World
  Eigen::Matrix<double, 3, 3> R_CI_; ///< Rot Camera->IMU
  Eigen::Matrix<double, 3, 3> R_WV_; ///< Rot World->Vision

  bool initialized_;
  bool predictionMade_;

  /// enables internal state predictions for log replay
  /**
   * used to determine if internal states get overwritten by the external
   * state prediction (online) or internal state prediction is performed
   * for log replay, when the external prediction is not available.
   */
  bool data_playback_;

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

  // dynamic reconfigure
  ReconfigureServer *reconfServer_;
  typedef boost::function<void(msf_core::MSF_CoreConfig& config, uint32_t level)> CallbackType;
  std::vector<CallbackType> callbacks_;

  /// propagates the state with given dt
  void propagateState(const double dt);

  /// propagets the error state covariance
  void predictProcessCovariance(const double dt);

  /// applies the correction
  bool applyCorrection(unsigned char idx_delaystate, const ErrorState & res_delayed, double fuzzythres = 0.1);

  /// propagate covariance to a given index in the ringbuffer
  void propPToIdx(unsigned char idx);

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

  /// gets called by dynamic reconfigure and calls all registered callbacks in callbacks_
  void Config(msf_core::MSF_CoreConfig &config, uint32_t level);

  /// handles the dynamic reconfigure for ssf_core
  void DynConfig(msf_core::MSF_CoreConfig &config, uint32_t level);

  /// computes the median of a given vector
  double getMedian(const Eigen::Matrix<double, nBuff_, 1> & data);

public:
  // some header implementations

  /// main update routine called by a given sensor
  template<class H_type, class Res_type, class R_type>
    bool applyMeasurement(unsigned char idx_delaystate, const Eigen::MatrixBase<H_type>& H_delayed,
                          const Eigen::MatrixBase<Res_type> & res_delayed, const Eigen::MatrixBase<R_type>& R_delayed,
                          double fuzzythres = 0.1)
    {
      EIGEN_STATIC_ASSERT_FIXED_SIZE(H_type);
      EIGEN_STATIC_ASSERT_FIXED_SIZE(R_type);

      // get measurements
      if (!predictionMade_)
        return false;

      // make sure we have correctly propagated cov until idx_delaystate
      propPToIdx(idx_delaystate);

      R_type S;
      Eigen::Matrix<double, N_STATE, R_type::RowsAtCompileTime> K;
      ErrorStateCov & P = StateBuffer_[idx_delaystate].P_;

      S = H_delayed * StateBuffer_[idx_delaystate].P_ * H_delayed.transpose() + R_delayed;
      K = P * H_delayed.transpose() * S.inverse();

      correction_ = K * res_delayed;
      const ErrorStateCov KH = (ErrorStateCov::Identity() - K * H_delayed);
      P = KH * P * KH.transpose() + K * R_delayed * K.transpose();

      // make sure P stays symmetric
      P = 0.5 * (P + P.transpose());

      return applyCorrection(idx_delaystate, correction_, fuzzythres);
    }

  /// registers dynamic reconfigure callbacks
  template<class T>
    void registerCallback(void(T::*cb_func)(msf_core::MSF_CoreConfig& config, uint32_t level), T* p_obj)
    {
      callbacks_.push_back(boost::bind(cb_func, p_obj, _1, _2));
    }
};

};// end namespace

#endif /* SSF_CORE_H_ */
