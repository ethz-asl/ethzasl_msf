/*
 * Copyright (C) 2014 Simon Lynen, ASL, ETH Zurich, Switzerland
 * You can contact the author at <slynen at ethz dot ch>
 * Copyright (C) 2014 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * You can contact the author at <acmarkus at ethz dot ch>
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

#include <stdexcept>
#include <msf_simulate/msf_statedef.hpp>
#include <msf_simulate/msf_simulator.h>

namespace msf_simulate {

MSFSimulator::MSFSimulator(const MSFSimulatorOptions& options)
    : options_(options) {
  generateRandomTrajectory();
  optimization_done_ = false;
}

MSFSimulator::MSFSimulator(const std::vector<Vertex4D>& sv,
                           const MSFSimulatorOptions& options)
    : options_(options),
      sv_(sv) {
  optimization_done_ = false;
}

MSFSimulator::~MSFSimulator() { }

void MSFSimulator::updatePathWithConstraint(int continuity,
                                            const Vertex1D & max_p,
                                            const Vertex1D & max_yaw,
                                            double time_multiplier,
                                            double tol) {
  path_.optimizeWithTime(sv_, continuity, max_p, max_yaw, time_multiplier, tol);
  optimization_done_ = true;
}

void MSFSimulator::generateRandomTrajectory() {

  Eigen::Matrix<double, 4, 1> tmp;

  sv_.push_back(Vertex4D(10, 2, 4));

  sv_.push_back(Vertex4D(10, 2));
  sv_.back().addConstraint(0, (tmp << 5, 0, 2, 0).finished());

  sv_.push_back(Vertex4D(2, 2));
  sv_.back().addConstraint(0, (tmp << 5, 5, 3, 0).finished());
  sv_.back().addConstraint(DerivativesP::v, (tmp << 0, 0, 0, 0).finished());

  sv_.push_back(Vertex4D(2, 2));
  sv_.back().addConstraint(0, (tmp << 0, 5, 2, 0).finished());

  sv_.push_back(Vertex4D(10, 2, 4));
  sv_.back().addConstraint(0, (tmp << -5, 5, 2, 0).finished());
}

}  // namespace msf_simulate
