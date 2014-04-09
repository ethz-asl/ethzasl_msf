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

#include <msf_core/testing_entrypoint.h>
#include <msf_simulate/msf_simulator.h>

TEST(MSF_Simulator, TrajectoryGeneration) {
  msf_simulate::MSFSimulatorOptions options;
  msf_simulate::MSFSimulator simulator(options);

  Vertex1D cp(1, 1);
  cp.addConstraint(1, 3);

  simulator.updatePathWithConstraint(DerivativesP::s, cp, cp);

  msf_simulate::MSFSimulator::MotionVector states;

  simulator.getMotion(states);

  std::cout << "Got " << states.size() << " states" << std::endl;

  // TODO(slynen): Add actual tests.
}
