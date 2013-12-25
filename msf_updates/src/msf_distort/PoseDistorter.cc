/*
 * Copyright (C) 2012-2013 Simon Lynen, ASL, ETH Zurich, Switzerland
 * You can contact the author at <slynen at ethz dot ch>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <msf_updates/PoseDistorter.h>
#include <msf_core/eigen_utils.h>
#include <ros/ros.h>

namespace msf_updates {

PoseDistorter::PoseDistorter(const Eigen::Vector3d& meanposdrift,
                             const Eigen::Vector3d& stddevposdrift,
                             const Eigen::Vector3d& meanattdrift,
                             const Eigen::Vector3d& stddevattdrift,
                             const double meanscaledrift,
                             const double stddevscaledrift)
    : gen_(rd_()) {
  posdrift_.setZero();
  attdrift_.setIdentity();
  scaledrift_ = 1;
  for (int i = 0; i < 3; ++i) {
    d_pos_[i] = distribution_t(meanposdrift(i), stddevposdrift(i));
    d_att_[i] = distribution_t(meanattdrift(i), stddevattdrift(i));
  }
  d_scale = distribution_t(meanscaledrift, stddevscaledrift);

  MSF_WARN_STREAM(
      "Distortion:\nPosition: Mean:"<<meanposdrift.transpose()<<" stddev: "<<stddevposdrift.transpose()<< "\nAttitude: Mean:"<<meanattdrift.transpose()<<" stddev: "<<stddevattdrift.transpose()<< "\nScale: Mean:"<<meanscaledrift<<" stddev: "<<stddevscaledrift);

}

PoseDistorter::~PoseDistorter() {

}

void PoseDistorter::Distort(Eigen::Vector3d& pos, double dt) {
  //calculate distortions
  Eigen::Vector3d deltapos;
  deltapos << d_pos_[0](gen_) * dt, d_pos_[1](gen_) * dt, d_pos_[2](gen_) * dt;

  double deltascale = d_scale(gen_) * dt;

  //add to rand walk
  posdrift_ += deltapos;
  scaledrift_ += deltascale;

  std::stringstream ss;
  ss << "Distort POS original: [" << pos.transpose() << "] posdrift: ["
      << posdrift_.transpose() << "] scale: " << scaledrift_;

  //augment state
  pos += posdrift_;
  pos *= scaledrift_;
  ss << " distorted: [" << pos.transpose() << "]";

  MSF_INFO_STREAM(ss.str());

}

void PoseDistorter::Distort(Eigen::Quaterniond& att, double dt) {
  //calculate distortions
  Eigen::Vector3d rpydist;
  rpydist << d_att_[0](gen_) * dt, d_att_[1](gen_) * dt, d_att_[2](gen_) * dt;
  Eigen::Quaterniond deltaquat = QuaternionFromSmallAngle(rpydist);

  deltaquat.normalize();

  //add to rand walk
  attdrift_ *= deltaquat;

  //augment state
  att *= attdrift_;
  MSF_INFO_STREAM(
      "Distort att:  ["<<attdrift_.w()<<", "<<attdrift_.x()<<", "<<attdrift_.y()<<", "<<attdrift_.z()<<"]");

}

void PoseDistorter::Distort(Eigen::Vector3d& pos, Eigen::Quaterniond& att,
                            double dt) {
  Distort(pos, dt);
  Distort(att, dt);
}

} /* namespace msf_updates */
