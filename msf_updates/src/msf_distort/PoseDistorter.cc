/*

Copyright (c) 2013, Simon Lynen, ASL, ETH Zurich, Switzerland
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

#include <msf_updates/PoseDistorter.h>
#include <msf_core/eigen_utils.h>
#include <ros/ros.h>

namespace msf_updates
{

PoseDistorter::PoseDistorter(const Eigen::Vector3d& meanposdrift, const Eigen::Vector3d& stddevposdrift, const Eigen::Vector3d& meanattdrift, const Eigen::Vector3d& stddevattdrift, const double meanscaledrift, const double stddevscaledrift):
                gen_(rd_())
{
  posdrift_.setZero();
  attdrift_.setIdentity();
  scaledrift_ = 1;
  for(int i = 0 ; i<3 ; ++i){
    d_pos_[i] = distribution_t(meanposdrift(i), stddevposdrift(i));
    d_att_[i] = distribution_t(meanattdrift(i), stddevattdrift(i));
  }
  d_scale = distribution_t(meanscaledrift, stddevscaledrift);

  ROS_WARN_STREAM("Distortion:\nPosition: Mean:"<<meanposdrift.transpose()<<" stddev: "<<stddevposdrift.transpose()<<
                  "\nAttitude: Mean:"<<meanattdrift.transpose()<<" stddev: "<<stddevattdrift.transpose()<<
                  "\nScale: Mean:"<<meanscaledrift<<" stddev: "<<stddevscaledrift);


}

PoseDistorter::~PoseDistorter()
{

}

void PoseDistorter::distort(Eigen::Vector3d& pos, double dt){
  //calculate distortions
  Eigen::Vector3d deltapos;
  deltapos << d_pos_[0](gen_) * dt, d_pos_[1](gen_) * dt, d_pos_[2](gen_) * dt;

  double deltascale = d_scale(gen_) * dt;

  //add to rand walk
  posdrift_ += deltapos;
  scaledrift_ += deltascale;

  std::stringstream ss;
  ss<<"Distort POS original: ["<<pos.transpose()<<"] posdrift: ["<<posdrift_.transpose()<<"] scale: "<<scaledrift_;

  //augment state
  pos += posdrift_;
  pos *= scaledrift_;
  ss<<" distorted: ["<<pos.transpose()<<"]";

  ROS_INFO_STREAM(ss.str());

}

void PoseDistorter::distort(Eigen::Quaterniond& att, double dt){
  //calculate distortions
  Eigen::Vector3d rpydist;
  rpydist << d_att_[0](gen_) * dt, d_att_[1](gen_) * dt, d_att_[2](gen_) * dt;
  Eigen::Quaterniond deltaquat = quaternionFromSmallAngle(rpydist);

  deltaquat.normalize();

  //add to rand walk
  attdrift_ *= deltaquat;

  //augment state
  att *= attdrift_;
  ROS_INFO_STREAM("Distort att:  ["<<attdrift_.w()<<", "<<attdrift_.x()<<", "<<attdrift_.y()<<", "<<attdrift_.z()<<"]");

}

void PoseDistorter::distort(Eigen::Vector3d& pos, Eigen::Quaterniond& att, double dt){
  distort(pos, dt);
  distort(att, dt);
}

} /* namespace msf_updates */
