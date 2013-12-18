/*
 * MSFSimulatorOptions.h
 *
 *  Created on: Jun 30, 2013
 *      Author: slynen
 */

#ifndef MSF_SIMULATOR_OPTIONS_H_
#define MSF_SIMULATOR_OPTIONS_H_

namespace msf_simulate{

struct MSFSimulatorOptions{

  MSFSimulatorOptions(){
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
}
#endif /* MSF_SIMULATOR_OPTIONS_H_ */
