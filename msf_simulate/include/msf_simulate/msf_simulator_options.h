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

#ifndef MSF_SIMULATOR_OPTIONS_H_
#define MSF_SIMULATOR_OPTIONS_H_

namespace msf_simulate {

struct MSFSimulatorOptions {

  MSFSimulatorOptions() {
    accelerometer_bias_.setZero();
    gyroscope_bias_.setZero();

    accelerometer_noise_sigma_.setZero();
    gyroscope_noise_sigma_.setZero();

    imu_rate_hz_ = 100;

    pose_rate_hz_ = 0;
    position_rate_hz_ = 0;
  }

  Eigen::Matrix<double, 3, 1> accelerometer_bias_;
  Eigen::Matrix<double, 3, 1> gyroscope_bias_;

  Eigen::Matrix<double, 3, 1> accelerometer_noise_sigma_;
  Eigen::Matrix<double, 3, 1> gyroscope_noise_sigma_;

  int imu_rate_hz_;

  int pose_rate_hz_;
  int position_rate_hz_;

};
}  // namespace msf_simulate
#endif  // MSF_SIMULATOR_OPTIONS_H_
