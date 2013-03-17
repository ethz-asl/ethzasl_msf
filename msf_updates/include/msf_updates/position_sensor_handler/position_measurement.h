/*

 Copyright (c) 2010, Stephan Weiss, ASL, ETH Zurich, Switzerland
 You can contact the author at <stephan dot weiss at ieee dot org>
 Copyright (c) 2012, Simon Lynen, ASL, ETH Zurich, Switzerland
 You can contact the author at <slynen at ethz dot ch>
 Copyright (c) 2012, Markus Achtelik, ASL, ETH Zurich, Switzerland
 You can contact the author at <acmarkus at ethz dot ch>

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

#ifndef POSITION_MEASUREMENT_HPP_
#define POSITION_MEASUREMENT_HPP_

#include <msf_core/msf_measurement.h>
#include <msf_core/msf_core.h>
#include <msf_core/eigen_utils.h>

namespace msf_updates{
namespace position_measurement{
enum
{
  nMeasurements = 3
};
/**
 * \brief a measurement as provided by a pose tracking algorithm
 */
typedef msf_core::MSF_Measurement<geometry_msgs::PointStamped, nMeasurements, msf_updates::EKFState> PositionMeasurementBase;
struct PositionMeasurement : public PositionMeasurementBase
{
private:
  typedef PositionMeasurementBase Measurement_t;
  typedef Measurement_t::Measurement_ptr measptr_t;

  virtual void makeFromSensorReadingImpl(measptr_t msg)
  {

    Eigen::Matrix<double, nMeasurements, msf_core::MSF_Core<msf_updates::EKFState>::nErrorStatesAtCompileTime> H_old;
    Eigen::Matrix<double, nMeasurements, 1> r_old;

    H_old.setZero();
    R_.setZero(); //TODO:remove later, already done in ctor of base

    // get measurement
    z_p_ = Eigen::Matrix<double, 3, 1>(msg->point.x, msg->point.y, msg->point.z);

    //    if (fixed_covariance_)//  take fix covariance from reconfigure GUI
    //    {

    const double s_zp = n_zp_ * n_zp_;
    R_ = (Eigen::Matrix<double, nMeasurements, 1>() << s_zp, s_zp, s_zp).finished().asDiagonal();

    //    }else{// take covariance from sensor
    //
    //      R_.block<3, 3>(0, 0) = Eigen::Matrix<double, 3, 3>(&msg->covariance[0]);
    //
    //      if(msg->header.seq % 100 == 0){ //only do this check from time to time
    //        if(R_.block<3, 3>(0, 0).determinant() < 0)
    //          ROS_WARN_STREAM_THROTTLE(60,"The covariance matrix you provided for the position sensor is not positive definite");
    //      }
    //    }
  }
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Matrix<double, 3, 1> z_p_; /// position measurement
  double n_zp_; /// position measurement noise

  bool fixed_covariance_;
  int fixedstates_;

  typedef msf_updates::EKFState EKFState_T;
  typedef EKFState_T::StateSequence_T StateSequence_T;
  typedef EKFState_T::StateDefinition_T StateDefinition_T;
  virtual ~PositionMeasurement()
  {
  }
  PositionMeasurement(double n_zp, bool fixed_covariance, bool isabsoluteMeasurement, int sensorID, int fixedstates) :
    PositionMeasurementBase(isabsoluteMeasurement, sensorID),
    n_zp_(n_zp), fixed_covariance_(fixed_covariance), fixedstates_(fixedstates)
  {
  }
  virtual std::string type(){
    return "position";
  }

  virtual void calculateH(boost::shared_ptr<EKFState_T> state_in, Eigen::Matrix<double, nMeasurements, msf_core::MSF_Core<EKFState_T>::nErrorStatesAtCompileTime>& H){
    const EKFState_T& state = *state_in; //get a const ref, so we can read core states

    H.setZero();

    // get rotation matrices
    Eigen::Matrix<double, 3, 3> C_q = state.get<StateDefinition_T::q>().conjugate().toRotationMatrix();

    // preprocess for elements in H matrix

    Eigen::Matrix<double, 3, 3> p_prism_imu_sk = skew(state.get<StateDefinition_T::p_ip>());

    //get indices of states in error vector
    enum{
      idxstartcorr_p_ = msf_tmp::getStartIndexInCorrection<StateSequence_T, StateDefinition_T::p>::value,
      idxstartcorr_v_ = msf_tmp::getStartIndexInCorrection<StateSequence_T, StateDefinition_T::v>::value,
      idxstartcorr_q_ = msf_tmp::getStartIndexInCorrection<StateSequence_T, StateDefinition_T::q>::value,
      idxstartcorr_p_pi_ = msf_tmp::getStartIndexInCorrection<StateSequence_T, StateDefinition_T::p_ip>::value,
    };

    bool fixed_p_pos_imu = (fixedstates_ & 1 << StateDefinition_T::p_ip);

    //clear crosscorrelations
    if(fixed_p_pos_imu) state_in->clearCrossCov<StateDefinition_T::p_ip>();

    // construct H matrix using H-blockx :-)
    // position:
    H.block<3, 3>(0, idxstartcorr_p_) = Eigen::Matrix<double, 3, 3>::Identity(); // p

    H.block<3, 3>(0, idxstartcorr_q_) = - C_q.transpose() * p_prism_imu_sk; // q

    H.block<3, 3>(0, idxstartcorr_p_pi_) = fixed_p_pos_imu ? Eigen::Matrix<double, 3, 3>::Zero() : (C_q.transpose()).eval(); //p_pos_imu_

  }

  /**
   * the method called by the msf_core to apply the measurement represented by this object
   */
  virtual void apply(boost::shared_ptr<EKFState_T> state_nonconst_new, msf_core::MSF_Core<EKFState_T>& core)
  {

    if(isabsolute_){//does this measurement refer to an absolute measurement, or is is just relative to the last measurement
      const EKFState_T& state = *state_nonconst_new; //get a const ref, so we can read core states
      // init variables
      Eigen::Matrix<double, nMeasurements, msf_core::MSF_Core<EKFState_T>::nErrorStatesAtCompileTime> H_new;
      Eigen::Matrix<double, nMeasurements, 1> r_old;

      calculateH(state_nonconst_new, H_new);

      // get rotation matrices
      Eigen::Matrix<double, 3, 3> C_q = state.get<StateDefinition_T::q>().conjugate().toRotationMatrix();

      // construct residuals
      // position
      r_old.block<3, 1>(0, 0) = z_p_ - (state.get<StateDefinition_T::p>() + C_q.transpose() * state.get<StateDefinition_T::p_ip>());

      if(!checkForNumeric(r_old, "r_old")){
        ROS_ERROR_STREAM("r_old: "<<r_old);
        ROS_WARN_STREAM("state: "<<const_cast<EKFState_T&>(state).toEigenVector().transpose());
      }
      if(!checkForNumeric(H_new, "H_old")){
        ROS_ERROR_STREAM("H_old: "<<H_new);
        ROS_WARN_STREAM("state: "<<const_cast<EKFState_T&>(state).toEigenVector().transpose());
      }
      if(!checkForNumeric(R_, "R_")){
        ROS_ERROR_STREAM("R_: "<<R_);
        ROS_WARN_STREAM("state: "<<const_cast<EKFState_T&>(state).toEigenVector().transpose());
      }

      // call update step in base class
      this->calculateAndApplyCorrection(state_nonconst_new, core, H_new, r_old, R_);

    }else{
      ROS_ERROR_STREAM_THROTTLE(1, "You chose to apply the position measurement as a relative quantitiy, which is currently not implemented.");
    }
  }
};

}
}
#endif /* POSITION_MEASUREMENT_HPP_ */
