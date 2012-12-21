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

#ifndef PRESSURE_MEASUREMENT_HPP_
#define PRESSURE_MEASUREMENT_HPP_

#include <msf_core/msf_measurement.hpp>
#include <msf_core/msf_core.hpp>

namespace pressure_measurement{
enum
{
  nMeasurements = 1
};
/**
 * \brief a measurement as provided by a pressure sensor
 */
struct PressureMeasurement : public msf_core::MSF_Measurement<asctec_hl_comm::mav_imu, nMeasurements, msf_updates::EKFState>
{
private:
  typedef msf_core::MSF_Measurement<asctec_hl_comm::mav_imu, nMeasurements, msf_updates::EKFState> Measurement_t;
  typedef Measurement_t::Measurement_ptr measptr_t;

  virtual void makeFromSensorReadingImpl(measptr_t msg)
  {

    Eigen::Matrix<double, nMeasurements, msf_core::MSF_Core<msf_updates::EKFState>::nErrorStatesAtCompileTime> H_old;
    Eigen::Matrix<double, nMeasurements, 1> r_old;

    H_old.setZero();
    R_.setZero(); //TODO:remove later, already done in ctor of base

    // get measurements
    z_p_ = Eigen::Matrix<double, 1, 1>::Constant(msg->height);

    const double s_zp = n_zp_ * n_zp_;
    R_ = (Eigen::Matrix<double, nMeasurements, 1>() << s_zp).finished().asDiagonal();
  }
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Matrix<double, 1, 1> z_p_; /// pressure measurement
  double n_zp_; /// pressure measurement noise

  typedef msf_updates::EKFState EKFState_T;
  typedef EKFState_T::StateDefinition_T StateDefinition_T;
  virtual ~PressureMeasurement()
  {
  }
  PressureMeasurement(double n_zp) :
    n_zp_(n_zp)
  {
  }
virtual std::string type(){
return "pressure";
  }
  /**
   * the method called by the msf_core to apply the measurement represented by this object
   */
  virtual void apply(boost::shared_ptr<EKFState_T> non_const_state, msf_core::MSF_Core<EKFState_T>& core)
  {

    // init variables
    Eigen::Matrix<double, nMeasurements, msf_core::MSF_Core<EKFState_T>::nErrorStatesAtCompileTime> H_old;
    Eigen::Matrix<double, nMeasurements, 1> r_old;

    H_old.setZero();

    if (non_const_state->time == -1)
    {
      ROS_WARN_STREAM("apply pose update was called with an invalid state");
      return;	// // early abort // //
    }

    const EKFState_T& state = *non_const_state;

    enum{
      idx_p = msf_tmp::getStartIndex<EKFState_T::StateSequence_T,
      typename msf_tmp::getEnumStateType<EKFState_T::StateSequence_T, StateDefinition_T::p>::value, msf_tmp::CorrectionStateLengthForType>::value,

      idx_b_p = msf_tmp::getStartIndex<EKFState_T::StateSequence_T,
      typename msf_tmp::getEnumStateType<EKFState_T::StateSequence_T, StateDefinition_T::p>::value, msf_tmp::CorrectionStateLengthForType>::value
    };

    // construct H matrix using H-blockx :-)
    // position:
    H_old.block<1, 1>(0, idx_p + 2)(0) = 1; // p_z
    //pressure bias
    H_old.block<1, 1>(0, idx_b_p)(0) = 1; //p_b

    // construct residuals
    // height
    r_old.block<1, 1>(0,0) = z_p_ - (state.get<StateDefinition_T::p>().block<1,1>(2, 0) - state.get<StateDefinition_T::b_p>());


    // call update step in base class
    this->calculateAndApplyCorrection(non_const_state, core, H_old, r_old, R_);

  }
};

}

#endif /* POSE_MEASUREMENT_HPP_ */
