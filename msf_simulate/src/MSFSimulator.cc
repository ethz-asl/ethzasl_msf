/*
 * MSFSimulator.cc
 *
 *  Created on: Jun 30, 2013
 *      Author: slynen
 */

#include <stdexcept>
#include <msf_simulate/msf_statedef.hpp>
#include <msf_simulate/MSFSimulator.h>

namespace msf_simulate {

MSFSimulator::MSFSimulator(const MSFSimulatorOptions& options)
: options_(options) {
  generateRandomTrajectory();
  optimization_done_ = false;
}

MSFSimulator::MSFSimulator(const std::vector<Vertex4D>& sv, const MSFSimulatorOptions& options)
: options_(options),
  sv_(sv) {
  optimization_done_ = false;
}

MSFSimulator::~MSFSimulator() {

}

void MSFSimulator::updatePathWithConstraint(int continuity, const Vertex1D & max_p, const Vertex1D & max_yaw,
                                            double time_multiplier, double tol) {
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



} /* namespace msf_simulate */
