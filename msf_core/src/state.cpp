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

#include <msf_core/state.h>

namespace msf_core
{

void State::reset(){
  // states varying during propagation
  p_.setZero();
  v_.setZero();
  q_.setIdentity();
  b_w_.setZero();
  b_a_.setZero();

  L_ = 1.0;
  q_wv_.setIdentity();
  q_ci_.setIdentity();
  p_ci_.setZero();

  w_m_.setZero();
  a_m_.setZero();

  q_int_.setIdentity();

  P_.setZero();
  time_ = 0;
}

void State::getPoseCovariance(geometry_msgs::PoseWithCovariance::_covariance_type & cov)
{
  assert(cov.size() == 36);

  for (int i = 0; i < 9; i++)
    cov[i / 3 * 6 + i % 3] = P_(i / 3 * N_STATE + i % 3);

  for (int i = 0; i < 9; i++)
    cov[i / 3 * 6 + (i % 3 + 3)] = P_(i / 3 * N_STATE + (i % 3 + 6));

  for (int i = 0; i < 9; i++)
    cov[(i / 3 + 3) * 6 + i % 3] = P_((i / 3 + 6) * N_STATE + i % 3);

  for (int i = 0; i < 9; i++)
    cov[(i / 3 + 3) * 6 + (i % 3 + 3)] = P_((i / 3 + 6) * N_STATE + (i % 3 + 6));
}

void State::toPoseMsg(geometry_msgs::PoseWithCovarianceStamped & pose)
{
  eigen_conversions::vector3dToPoint(p_, pose.pose.pose.position);
  eigen_conversions::quaternionToMsg(q_, pose.pose.pose.orientation);
  getPoseCovariance(pose.pose.covariance);
}

void State::toExtStateMsg(sensor_fusion_comm::ExtState & state)
{
  eigen_conversions::vector3dToPoint(p_, state.pose.position);
  eigen_conversions::quaternionToMsg(q_, state.pose.orientation);
  eigen_conversions::vector3dToPoint(v_, state.velocity);
}

void State::toStateMsg(sensor_fusion_comm::DoubleArrayStamped & state)
{
  state.data[0] = p_[0];
  state.data[1] = p_[1];
  state.data[2] = p_[2];
  state.data[3] = v_[0];
  state.data[4] = v_[1];
  state.data[5] = v_[2];
  state.data[6] = q_.w();
  state.data[7] = q_.x();
  state.data[8] = q_.y();
  state.data[9] = q_.z();
  state.data[10] = b_w_[0];
  state.data[11] = b_w_[1];
  state.data[12] = b_w_[2];
  state.data[13] = b_a_[0];
  state.data[14] = b_a_[1];
  state.data[15] = b_a_[2];
  state.data[16] = L_;
  state.data[17] = q_wv_.w();
  state.data[18] = q_wv_.x();
  state.data[19] = q_wv_.y();
  state.data[20] = q_wv_.z();
  state.data[21] = q_ci_.w();
  state.data[22] = q_ci_.x();
  state.data[23] = q_ci_.y();
  state.data[24] = q_ci_.z();
  state.data[25] = p_ci_[0];
  state.data[26] = p_ci_[1];
  state.data[27] = p_ci_[2];
}

}; // end namespace ssf_core
