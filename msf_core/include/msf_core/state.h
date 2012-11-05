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

#ifndef STATE_H_
#define STATE_H_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <msf_core/eigen_conversions.h>
#include <sensor_fusion_comm/ExtState.h>
#include <sensor_fusion_comm/DoubleArrayStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#define N_STATE 25 /// error state size

namespace msf_core
{
/**
 * This class defines the state, its associated error state covarinace and the
 * system inputs. The values in the braces determine the state's position in the
 * state vector / error state vector.
 */
class State
{
public:
  // states varying during propagation
  Eigen::Matrix<double, 3, 1> p_;         ///< position (IMU centered)          (0-2 / 0-2)
  Eigen::Matrix<double, 3, 1> v_;         ///< velocity                         (3- 5 / 3- 5)
  Eigen::Quaternion<double> q_;           ///< attitude                         (6- 9 / 6- 8)
  Eigen::Matrix<double, 3, 1> b_w_;       ///< gyro biases                      (10-12 / 9-11)
  Eigen::Matrix<double, 3, 1> b_a_;       ///< acceleration biases              (13-15 / 12-14)

  // states not varying during propagation
  double L_;                              ///< visual scale                     (16 / 15)
  Eigen::Quaternion<double> q_wv_;        ///< vision-world attitude drift      (17-20 / 16-18)
  Eigen::Quaternion<double> q_ci_;        ///< camera-imu attitude calibration  (21-24 / 19-21)
  Eigen::Matrix<double, 3, 1> p_ci_;      ///< camera-imu position calibration  (25-27 / 22-24)

  // system inputs
  Eigen::Matrix<double,3,1> w_m_;         ///< angular velocity from IMU
  Eigen::Matrix<double,3,1> a_m_;         ///< acceleration from IMU

  Eigen::Quaternion<double> q_int_;       ///< this is the integrated ang. vel. no corrections applied, to use for delta rot in external algos...

  Eigen::Matrix<double, N_STATE, N_STATE> P_;///< error state covariance

  double time_; ///< time of this state estimate

  /// resets the state
  /**
   * 3D vectors: 0; quaternion: unit quaternion; scale: 1; time:0; Error covariance: zeros
   */
  void reset();

  /// writes the covariance corresponding to position and attitude to cov
  void getPoseCovariance(geometry_msgs::PoseWithCovariance::_covariance_type & cov);

  /// assembles a PoseWithCovarianceStamped message from the state
  /** it does not set the header */
  void toPoseMsg(geometry_msgs::PoseWithCovarianceStamped & pose);

  /// assembles an ExtState message from the state
  /** it does not set the header */
  void toExtStateMsg(sensor_fusion_comm::ExtState & state);

  /// assembles a DoubleArrayStamped message from the state
  /** it does not set the header */
  void toStateMsg(sensor_fusion_comm::DoubleArrayStamped & state);



};

}

#endif /* STATE_H_ */
