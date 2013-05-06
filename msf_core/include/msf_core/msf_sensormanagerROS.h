/*

 Copyright (c) 2012, Simon Lynen, ASL, ETH Zurich, Switzerland
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

#ifndef SENSORMANAGERROS_H
#define SENSORMANAGERROS_H

#include <msf_core/MSF_CoreConfig.h>
#include <msf_core/msf_sensormanager.h>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

namespace msf_core {

typedef dynamic_reconfigure::Server<msf_core::MSF_CoreConfig> ReconfigureServer;

/** \class MSF_SensorManagerROS
 * \brief Abstract class defining user configurable calculations for the msf_core with ROS interfaces
 */
template<typename EKFState_T>
struct MSF_SensorManagerROS : public msf_core::MSF_SensorManager<EKFState_T> {
 protected:
  msf_core::MSF_CoreConfig config_;  ///< dynamic reconfigure config
 private:

  ReconfigureServer *reconfServer_;  ///< dynamic reconfigure server

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MSF_SensorManagerROS(ros::NodeHandle pnh = ros::NodeHandle("~core")) {
    reconfServer_ = new ReconfigureServer(pnh);
    ReconfigureServer::CallbackType f = boost::bind(
        &MSF_SensorManagerROS::Config, this, _1, _2);
    reconfServer_->setCallback(f);
  }

  virtual ~MSF_SensorManagerROS() {
    delete reconfServer_;
  }

  /**
   * \brief gets called by the internal callback caller
   */
  virtual void coreConfig(msf_core::MSF_CoreConfig &config, uint32_t level) {
    UNUSED(config);
    UNUSED(level);
  }

  /**
   * \brief gets called by dynamic reconfigure and calls all registered callbacks in callbacks_
   */
  void Config(msf_core::MSF_CoreConfig &config, uint32_t level) {
    config_ = config;
    coreConfig(config, level);
  }

  //parameter getters
  virtual bool getParam_fixed_bias() const {
    return config_.core_fixed_bias;
  }
  virtual double getParam_noise_acc() const {
    return config_.core_noise_acc;
  }
  virtual double getParam_noise_accbias() const {
    return config_.core_noise_accbias;
  }
  virtual double getParam_noise_gyr() const {
    return config_.core_noise_gyr;
  }
  virtual double getParam_noise_gyrbias() const {
    return config_.core_noise_gyrbias;
  }
  virtual double getParam_fuzzythres() const {
    return 0.1;
  }

};

}
#endif /* SENSORMANAGERROS_H */
