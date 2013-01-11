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

#ifndef POSE_MEASUREMENT_HPP_
#define POSE_MEASUREMENT_HPP_

#include <msf_core/msf_measurement.h>
#include <msf_core/msf_core.h>

namespace pose_measurement{
enum
{
  nMeasurements = 7
};
/**
 * \brief a measurement as provided by a pose tracking algorithm
 */
struct PoseMeasurement : public msf_core::MSF_Measurement<geometry_msgs::PoseWithCovarianceStamped, nMeasurements, msf_updates::EKFState>
{
private:
  typedef msf_core::MSF_Measurement<geometry_msgs::PoseWithCovarianceStamped, nMeasurements, msf_updates::EKFState> Measurement_t;
  typedef Measurement_t::Measurement_ptr measptr_t;

  virtual void makeFromSensorReadingImpl(measptr_t msg)
  {

    Eigen::Matrix<double, nMeasurements, msf_core::MSF_Core<msf_updates::EKFState>::nErrorStatesAtCompileTime> H_old;
    Eigen::Matrix<double, nMeasurements, 1> r_old;

    H_old.setZero();
    R_.setZero(); //TODO:remove later, already done in ctor of base

    // get measurements
    z_p_ = Eigen::Matrix<double, 3, 1>(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    z_q_ = Eigen::Quaternion<double>(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                     msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);

    if (fixed_covariance_)//  take fix covariance from reconfigure GUI
    {

      const double s_zp = n_zp_ * n_zp_;
      const double s_zq = n_zq_ * n_zq_;
      R_ = (Eigen::Matrix<double, nMeasurements, 1>() << s_zp, s_zp, s_zp, s_zq, s_zq, s_zq, 1e-6).finished().asDiagonal();

    }else{// take covariance from sensor

      R_.block<6, 6>(0, 0) = Eigen::Matrix<double, 6, 6>(&msg->pose.covariance[0]);

      if(msg->header.seq % 100 == 0){ //only do this check from time to time
        if(R_.block<6, 6>(0, 0).determinant()<=0)
          ROS_ERROR_STREAM("The covariance matrix you provided is not positive definite");
      }

      //clear cross-correlations between q and p
      R_.block<3, 3>(0, 3) = Eigen::Matrix<double, 3, 3>::Zero();
      R_.block<3, 3>(3, 0) = Eigen::Matrix<double, 3, 3>::Zero();
      R_(6, 6) = 1e-6; // q_vw yaw-measurement noise

      /*************************************************************************************/
      // use this if your pose sensor is ethzasl_ptam (www.ros.org/wiki/ethzasl_ptam)
      // ethzasl_ptam publishes the camera pose as the world seen from the camera
      if (!measurement_world_sensor_)
      {
        Eigen::Matrix<double, 3, 3> C_zq = z_q_.toRotationMatrix();
        z_q_ = z_q_.conjugate();
        z_p_ = -C_zq.transpose() * z_p_;

        Eigen::Matrix<double, 6, 6> C_cov(Eigen::Matrix<double, 6, 6>::Zero());
        C_cov.block<3, 3>(0, 0) = C_zq;
        C_cov.block<3, 3>(3, 3) = C_zq;

        R_.block<6, 6>(0, 0) = C_cov.transpose() * R_.block<6, 6>(0, 0) * C_cov;
      }
      /*************************************************************************************/
    }
  }
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Quaternion<double> z_q_; /// attitude measurement camera seen from world
  Eigen::Matrix<double, 3, 1> z_p_; /// position measurement camera seen from world
  double n_zp_, n_zq_; /// position and attitude measurement noise

  bool measurement_world_sensor_;
  bool fixed_covariance_;

  typedef msf_updates::EKFState EKFState_T;
  typedef EKFState_T::StateDefinition_T StateDefinition_T;
  virtual ~PoseMeasurement()
  {
  }
  PoseMeasurement(double n_zp, double n_zq, bool measurement_world_sensor, bool fixed_covariance) :
    n_zp_(n_zp), n_zq_(n_zq), measurement_world_sensor_(measurement_world_sensor), fixed_covariance_(fixed_covariance)
  {
  }
  virtual std::string type(){
    return "pose";
  }
  /**
   * the method called by the msf_core to apply the measurement represented by this object
   */
  virtual void apply(boost::shared_ptr<EKFState_T> const_state, msf_core::MSF_Core<EKFState_T>& core)
  {

    // init variables
    Eigen::Matrix<double, nMeasurements, msf_core::MSF_Core<EKFState_T>::nErrorStatesAtCompileTime> H_old;
    Eigen::Matrix<double, nMeasurements, 1> r_old;

    H_old.setZero();

    if (const_state->time == -1 || !const_state->checkStateForNumeric())
    {
      ROS_WARN_STREAM("apply pose update was called with an invalid state");
      return;	// // early abort // //
    }

    const EKFState_T& state = *const_state;

    // get rotation matrices
    Eigen::Matrix<double, 3, 3> C_wv = state.get<StateDefinition_T::q_wv>().conjugate().toRotationMatrix();
    Eigen::Matrix<double, 3, 3> C_q = state.get<StateDefinition_T::q>().conjugate().toRotationMatrix();
    Eigen::Matrix<double, 3, 3> C_ci = state.get<StateDefinition_T::q_ci>().conjugate().toRotationMatrix();

    // preprocess for elements in H matrix
    Eigen::Matrix<double, 3, 1> vecold;
    vecold = (state.get<StateDefinition_T::p>() + C_q.transpose() * state.get<StateDefinition_T::p_ci>()) * state.get<StateDefinition_T::L>();
    Eigen::Matrix<double, 3, 3> skewold = skew(vecold);

    Eigen::Matrix<double, 3, 3> pci_sk = skew(state.get<StateDefinition_T::p_ci>());

    // construct H matrix using H-blockx :-)
    // position:
    H_old.block<3, 3>(0, 0) = C_wv.transpose() * state.get<StateDefinition_T::L>()(0); // p
    H_old.block<3, 3>(0, 6) = -C_wv.transpose() * C_q.transpose() * pci_sk * state.get<StateDefinition_T::L>()(0); // q
    H_old.block<3, 1>(0, 15) = C_wv.transpose() * C_q.transpose() * state.get<StateDefinition_T::p_ci>()
                                        + C_wv.transpose() * state.get<StateDefinition_T::p>(); // L
    H_old.block<3, 3>(0, 16) = -C_wv.transpose() * skewold; // q_wv
    H_old.block<3, 3>(0, 22) = C_wv.transpose() * C_q.transpose() * state.get<StateDefinition_T::L>()(0); //p_ci

    // attitude
    H_old.block<3, 3>(3, 6) = C_ci; // q
    H_old.block<3, 3>(3, 16) = C_ci * C_q; // q_wv
    H_old.block<3, 3>(3, 19) = Eigen::Matrix<double, 3, 3>::Identity(); //q_ci
    H_old(6, 18) = 1.0; // fix vision world yaw drift because unobservable otherwise (see PhD Thesis)

    // construct residuals
    // position
    r_old.block<3, 1>(0, 0) = z_p_
        - C_wv.transpose() * (state.get<StateDefinition_T::p>() + C_q.transpose() * state.get<StateDefinition_T::p_ci>())
        * state.get<StateDefinition_T::L>();
    // attitude
    Eigen::Quaternion<double> q_err;
    q_err = (state.get<StateDefinition_T::q_wv>() * state.get<StateDefinition_T::q>() * state.get<StateDefinition_T::q_ci>()).conjugate() * z_q_;
    r_old.block<3, 1>(3, 0) = q_err.vec() / q_err.w() * 2;
    // vision world yaw drift
    q_err = state.get<StateDefinition_T::q_wv>();
    r_old(6, 0) = -2 * (q_err.w() * q_err.z() + q_err.x() * q_err.y())
                                        / (1 - 2 * (q_err.y() * q_err.y() + q_err.z() * q_err.z()));

    if(!checkForNumeric((double*)&r_old, nMeasurements, "r_old")){
      ROS_ERROR_STREAM("r_old: "<<r_old);
      ROS_WARN_STREAM("state: "<<const_cast<EKFState_T&>(state).toEigenVector().transpose());
    }
    if(!checkForNumeric((double*)&H_old, H_old.RowsAtCompileTime * H_old.ColsAtCompileTime, "H_old")){
      ROS_ERROR_STREAM("H_old: "<<H_old);
      ROS_WARN_STREAM("state: "<<const_cast<EKFState_T&>(state).toEigenVector().transpose());
    }
    if(!checkForNumeric((double*)&R_, R_.RowsAtCompileTime * R_.ColsAtCompileTime, "R_")){
      ROS_ERROR_STREAM("R_: "<<R_);
      ROS_WARN_STREAM("state: "<<const_cast<EKFState_T&>(state).toEigenVector().transpose());
    }

    // call update step in base class
    this->calculateAndApplyCorrection(const_state, core, H_old, r_old, R_);

  }
};

}

#endif /* POSE_MEASUREMENT_HPP_ */
