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
#ifndef POSEDISTORTER_H_
#define POSEDISTORTER_H_

#include <Eigen/Dense>
#include <random>
#include <cmath>
#include <msf_core/msf_types.h>
#include <msf_core/msf_fwds.h>

namespace msf_updates {

class PoseDistorter {
 public:
  typedef std::normal_distribution<> distribution_t;
  typedef shared_ptr<PoseDistorter> Ptr;
 private:
  Eigen::Vector3d posdrift_;
  Eigen::Quaterniond attdrift_;
  double scaledrift_;

  std::random_device rd_;
  std::mt19937 gen_;

  distribution_t d_pos_[3];
  distribution_t d_att_[3];
  distribution_t d_scale;
 public:
  PoseDistorter(const Eigen::Vector3d& meanposdrift,
                const Eigen::Vector3d& stddevposdrift,
                const Eigen::Vector3d& meanattdrift,
                const Eigen::Vector3d& stddevattdrift,
                const double meanscaledrift, const double stddevscaledrift);
  void Distort(Eigen::Vector3d& pos, Eigen::Quaterniond& att, double dt);
  void Distort(Eigen::Vector3d& pos, double dt);
  void Distort(Eigen::Quaterniond& att, double dt);
  virtual ~PoseDistorter();
};

}  // namespace msf_updates
#endif  // POSEDISTORTER_H_
