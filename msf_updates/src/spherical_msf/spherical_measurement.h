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
#include <geometry_msgs/PointStamped.h>


namespace msf_spherical_position{
enum
{
  N_ANGLE_MEASUREMENTS = 2, N_DISTANCE_MEASUREMENTS=1
};


/**
 * \brief an angle measurement from a spherical position sensor
 */
typedef msf_core::MSF_Measurement<geometry_msgs::PointStamped, N_ANGLE_MEASUREMENTS, msf_updates::EKFState> AngleMeasurementBase;
struct AngleMeasurement : public AngleMeasurementBase
{
private:
  typedef AngleMeasurementBase Measurement_t;
  typedef Measurement_t::Measurement_ptr measptr_t;

  virtual void makeFromSensorReadingImpl(measptr_t msg)
  {

    Eigen::Matrix<double, N_ANGLE_MEASUREMENTS, msf_core::MSF_Core<msf_updates::EKFState>::nErrorStatesAtCompileTime> H_old;
    Eigen::Matrix<double, N_ANGLE_MEASUREMENTS, 1> r_old;

    H_old.setZero();
    R_.setZero(); //TODO:remove later, already done in ctor of base

    // get measurement
    z_a_ << msg->point.x, msg->point.y;

    if (fixed_covariance_)//  take fix covariance from reconfigure GUI
    {

      const double s_zp = n_za_ * n_za_;
      R_ = (Eigen::Matrix<double, N_ANGLE_MEASUREMENTS, 1>() << s_zp, s_zp).finished().asDiagonal();

    }else{// take covariance from sensor

//      R_.block<3, 3>(0, 0) = msf_core::Matrix3(&msg->covariance[0]);
//
//      if(msg->header.seq % 100 == 0){ //only do this check from time to time
//        if(R_.block<3, 3>(0, 0).determinant() < -0.01)
//          ROS_WARN_STREAM_THROTTLE(60,"The covariance matrix you provided for the position sensor is not positive definite");
//      }
      ROS_WARN_STREAM_THROTTLE(60,"using non-fixed covariance not implemented yet");
    }
  }
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  msf_core::Vector2 z_a_; /// position measurement
  double n_za_; /// position measurement noise

  bool fixed_covariance_;
  int fixedstates_;

  typedef msf_updates::EKFState EKFState_T;
  typedef EKFState_T::StateSequence_T StateSequence_T;
  typedef EKFState_T::StateDefinition_T StateDefinition_T;
  virtual ~AngleMeasurement()
  {
  }
  AngleMeasurement(double n_za, bool fixed_covariance, bool isabsoluteMeasurement, int sensorID, int fixedstates) :
    AngleMeasurementBase(isabsoluteMeasurement, sensorID),
    n_za_(n_za), fixed_covariance_(fixed_covariance), fixedstates_(fixedstates)
  {
  }
  virtual std::string type(){
    return "angle";
  }

  virtual void calculateH(boost::shared_ptr<EKFState_T> state_in, Eigen::Matrix<double, N_ANGLE_MEASUREMENTS, msf_core::MSF_Core<EKFState_T>::nErrorStatesAtCompileTime>& H){
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

    // TODO: @georg: implement your H matrix for the angles here
//    H.block<2, 3>(0, idxstartcorr_p_) = Eigen::Matrix<double, 3, 3>::Identity(); // p
//    H.block<2, 3>(0, idxstartcorr_q_) = - C_q.transpose() * p_prism_imu_sk; // q
//    H.block<2, 3>(0, idxstartcorr_p_pi_) = fixed_p_pos_imu ? Eigen::Matrix<double, 3, 3>::Zero() : (C_q.transpose()).eval(); //p_pos_imu_

  }

  /**
   * the method called by the msf_core to apply the measurement represented by this object
   */
  virtual void apply(boost::shared_ptr<EKFState_T> state_nonconst_new, msf_core::MSF_Core<EKFState_T>& core)
  {

    if(isabsolute_){//does this measurement refer to an absolute measurement, or is is just relative to the last measurement
      const EKFState_T& state = *state_nonconst_new; //get a const ref, so we can read core states
      // init variables
      Eigen::Matrix<double, N_ANGLE_MEASUREMENTS, msf_core::MSF_Core<EKFState_T>::nErrorStatesAtCompileTime> H_new;
      Eigen::Matrix<double, N_ANGLE_MEASUREMENTS, 1> r_old;

      calculateH(state_nonconst_new, H_new);

      // get rotation matrices
      Eigen::Matrix<double, 3, 3> C_q = state.get<StateDefinition_T::q>().conjugate().toRotationMatrix();

      // construct residuals
      // TODO: @georg: implement your residual here
      r_old = z_a_ ;//- (state.get<StateDefinition_T::p>() + C_q.transpose() * state.get<StateDefinition_T::p_ip>());

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







/**
 * \brief a distance measurement from a spherical position sensor
 */
typedef msf_core::MSF_Measurement<geometry_msgs::PointStamped, N_DISTANCE_MEASUREMENTS, msf_updates::EKFState> DistanceMeasurementBase;
struct DistanceMeasurement : public DistanceMeasurementBase
{
private:
  typedef DistanceMeasurementBase Measurement_t;
  typedef Measurement_t::Measurement_ptr measptr_t;

  virtual void makeFromSensorReadingImpl(measptr_t msg)
  {

    Eigen::Matrix<double, N_DISTANCE_MEASUREMENTS, msf_core::MSF_Core<msf_updates::EKFState>::nErrorStatesAtCompileTime> H_old;
    Eigen::Matrix<double, N_DISTANCE_MEASUREMENTS, 1> r_old;

    H_old.setZero();
    R_.setZero(); //TODO:remove later, already done in ctor of base

    // get measurement
    z_d_ << msg->point.x, msg->point.y;

    if (fixed_covariance_)//  take fix covariance from reconfigure GUI
    {
      R_(0,0) = n_zd_ * n_zd_;
    }else{// take covariance from sensor

//      R_.block<3, 3>(0, 0) = msf_core::Matrix3(&msg->covariance[0]);
//
//      if(msg->header.seq % 100 == 0){ //only do this check from time to time
//        if(R_.block<3, 3>(0, 0).determinant() < -0.01)
//          ROS_WARN_STREAM_THROTTLE(60,"The covariance matrix you provided for the position sensor is not positive definite");
//      }
      ROS_WARN_STREAM_THROTTLE(60,"using non-fixed covariance not implemented yet");
    }
  }
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  msf_core::Vector1 z_d_; /// position measurement
  double n_zd_; /// position measurement noise

  bool fixed_covariance_;
  int fixedstates_;

  typedef msf_updates::EKFState EKFState_T;
  typedef EKFState_T::StateSequence_T StateSequence_T;
  typedef EKFState_T::StateDefinition_T StateDefinition_T;
  virtual ~DistanceMeasurement()
  {
  }
  DistanceMeasurement(double n_zd, bool fixed_covariance, bool isabsoluteMeasurement, int sensorID, int fixedstates) :
    DistanceMeasurementBase(isabsoluteMeasurement, sensorID),
    n_zd_(n_zd), fixed_covariance_(fixed_covariance), fixedstates_(fixedstates)
  {
  }
  virtual std::string type(){
    return "distance";
  }

  virtual void calculateH(boost::shared_ptr<EKFState_T> state_in, Eigen::Matrix<double, N_DISTANCE_MEASUREMENTS, msf_core::MSF_Core<EKFState_T>::nErrorStatesAtCompileTime>& H){
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

    // TODO: @georg: implement your H matrix for the distance here
//    H.block<1, 3>(0, idxstartcorr_p_) =  // p
//    H.block<1, 3>(0, idxstartcorr_q_) =  // q
//    H.block<1, 3>(0, idxstartcorr_p_pi_) =  //p_pos_imu_

  }

  /**
   * the method called by the msf_core to apply the measurement represented by this object
   */
  virtual void apply(boost::shared_ptr<EKFState_T> state_nonconst_new, msf_core::MSF_Core<EKFState_T>& core)
  {

    if(isabsolute_){//does this measurement refer to an absolute measurement, or is is just relative to the last measurement
      const EKFState_T& state = *state_nonconst_new; //get a const ref, so we can read core states
      // init variables
      Eigen::Matrix<double, N_DISTANCE_MEASUREMENTS, msf_core::MSF_Core<EKFState_T>::nErrorStatesAtCompileTime> H_new;
      Eigen::Matrix<double, N_DISTANCE_MEASUREMENTS, 1> r_old;

      calculateH(state_nonconst_new, H_new);

      // get rotation matrices
      Eigen::Matrix<double, 3, 3> C_q = state.get<StateDefinition_T::q>().conjugate().toRotationMatrix();

      // construct residuals
      // TODO: @georg: implement your residual here
      r_old = z_d_ ;//- (state.get<StateDefinition_T::p>() + C_q.transpose() * state.get<StateDefinition_T::p_ip>());

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

#endif /* POSITION_MEASUREMENT_HPP_ */
