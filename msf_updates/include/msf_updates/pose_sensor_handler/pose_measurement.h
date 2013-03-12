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
#include <msf_updates/PoseDistorter.h>

namespace msf_updates{
namespace pose_measurement{
enum
{
  nMeasurements = 7
};
/**
 * \brief a measurement as provided by a pose tracking algorithm
 */
typedef msf_core::MSF_Measurement<geometry_msgs::PoseWithCovarianceStamped, nMeasurements, msf_updates::EKFState> PoseMeasurementBase;
struct PoseMeasurement : public PoseMeasurementBase
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

    if(distorter_){
      static double tlast = 0;
      if(tlast != 0){
        double dt = time - tlast;
        distorter_->distort(z_p_, z_q_, dt);
      }
      tlast = time;
    }

    if (fixed_covariance_)//  take fix covariance from reconfigure GUI
    {

      const double s_zp = n_zp_ * n_zp_;
      const double s_zq = n_zq_ * n_zq_;
      R_ = (Eigen::Matrix<double, nMeasurements, 1>() << s_zp, s_zp, s_zp, s_zq, s_zq, s_zq, 1e-6).finished().asDiagonal();

    }else{// take covariance from sensor

      R_.block<6, 6>(0, 0) = Eigen::Matrix<double, 6, 6>(&msg->pose.covariance[0]);

      if(msg->header.seq % 100 == 0){ //only do this check from time to time
        if(R_.block<6, 6>(0, 0).determinant() < 0)
          ROS_WARN_STREAM_THROTTLE(60,"The covariance matrix you provided for the pose sensor is not positive definite: "<<(R_.block<6, 6>(0, 0)));
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
  msf_updates::PoseDistorter::Ptr distorter_;
  int fixedstates_;

  typedef msf_updates::EKFState EKFState_T;
  typedef EKFState_T::StateSequence_T StateSequence_T;
  typedef EKFState_T::StateDefinition_T StateDefinition_T;
  virtual ~PoseMeasurement()
  {
  }
  PoseMeasurement(double n_zp, double n_zq, bool measurement_world_sensor, bool fixed_covariance, bool isabsoluteMeasurement, int sensorID, int fixedstates, msf_updates::PoseDistorter::Ptr distorter = msf_updates::PoseDistorter::Ptr()) :
    PoseMeasurementBase(isabsoluteMeasurement, sensorID),
    n_zp_(n_zp), n_zq_(n_zq), measurement_world_sensor_(measurement_world_sensor), fixed_covariance_(fixed_covariance), distorter_(distorter), fixedstates_(fixedstates)
  {
  }
  virtual std::string type(){
    return "pose";
  }

  virtual void calculateH(boost::shared_ptr<EKFState_T> state_in, Eigen::Matrix<double, nMeasurements, msf_core::MSF_Core<EKFState_T>::nErrorStatesAtCompileTime>& H){
    const EKFState_T& state = *state_in; //get a const ref, so we can read core states

    H.setZero();

    // get rotation matrices
    Eigen::Matrix<double, 3, 3> C_wv = state.get<StateDefinition_T::q_wv>().conjugate().toRotationMatrix();
    Eigen::Matrix<double, 3, 3> C_q = state.get<StateDefinition_T::q>().conjugate().toRotationMatrix();
    Eigen::Matrix<double, 3, 3> C_ci = state.get<StateDefinition_T::q_ci>().conjugate().toRotationMatrix();

    // preprocess for elements in H matrix
    Eigen::Matrix<double, 3, 1> vecold;
    vecold = (state.get<StateDefinition_T::p>() + C_q.transpose() * state.get<StateDefinition_T::p_ci>()) * state.get<StateDefinition_T::L>();
    Eigen::Matrix<double, 3, 3> skewold = skew(vecold);

    Eigen::Matrix<double, 3, 3> pci_sk = skew(state.get<StateDefinition_T::p_ci>());

    //get indices of states in error vector
    enum{
      idxstartcorr_p_ = msf_tmp::getStartIndexInCorrection<StateSequence_T, StateDefinition_T::p>::value,
      idxstartcorr_v_ = msf_tmp::getStartIndexInCorrection<StateSequence_T, StateDefinition_T::v>::value,
      idxstartcorr_q_ = msf_tmp::getStartIndexInCorrection<StateSequence_T, StateDefinition_T::q>::value,
      idxstartcorr_L_ = msf_tmp::getStartIndexInCorrection<StateSequence_T, StateDefinition_T::L>::value,
      idxstartcorr_qwv_ = msf_tmp::getStartIndexInCorrection<StateSequence_T, StateDefinition_T::q_wv>::value,
      idxstartcorr_pvw_ = msf_tmp::getStartIndexInCorrection<StateSequence_T, StateDefinition_T::p_vw>::value,
      idxstartcorr_qci_ = msf_tmp::getStartIndexInCorrection<StateSequence_T, StateDefinition_T::q_ci>::value,
      idxstartcorr_pci_ = msf_tmp::getStartIndexInCorrection<StateSequence_T, StateDefinition_T::p_ci>::value,
    };

    //read the fixed states flags
    bool scalefix = (fixedstates_ & 1 << StateDefinition_T::L);
    bool calibposfix = (fixedstates_ & 1 << StateDefinition_T::p_ci);
    bool calibattfix = (fixedstates_ & 1 << StateDefinition_T::q_ci);
    bool driftvwattfix = (fixedstates_ & 1 << StateDefinition_T::q_wv);
    bool driftvwposfix = (fixedstates_ & 1 << StateDefinition_T::p_vw);

    //set crosscov to zero
    if(scalefix) state_in->clearCrossCov<StateDefinition_T::L>();
    if(calibposfix) state_in->clearCrossCov<StateDefinition_T::p_ci>();
    if(calibattfix) state_in->clearCrossCov<StateDefinition_T::q_ci>();
    if(driftvwattfix) state_in->clearCrossCov<StateDefinition_T::q_wv>();
    if(driftvwposfix) state_in->clearCrossCov<StateDefinition_T::p_vw>();

    // construct H matrix using H-blockx :-)
    // position:
    H.block<3, 3>(0, idxstartcorr_p_) = C_wv.transpose() * state.get<StateDefinition_T::L>()(0); // p

    H.block<3, 3>(0, idxstartcorr_q_) = -C_wv.transpose() * C_q.transpose() * pci_sk * state.get<StateDefinition_T::L>()(0); // q

    H.block<3, 1>(0, idxstartcorr_L_) = scalefix ? Eigen::Matrix<double, 3, 1>::Constant(0) :
        (C_wv.transpose() * C_q.transpose() * state.get<StateDefinition_T::p_ci>()
        + C_wv.transpose() * state.get<StateDefinition_T::p>() + (driftvwposfix ? Eigen::Matrix<double, 3, 1>::Constant(0) :
            state.get<StateDefinition_T::p_vw>()).eval()).eval(); // L

    H.block<3, 3>(0, idxstartcorr_qwv_) = driftvwposfix ? Eigen::Matrix<double, 3, 3>::Constant(0) :
        (-C_wv.transpose() * skewold).eval(); // q_wv

    H.block<3, 3>(0, idxstartcorr_pci_) = calibposfix ? Eigen::Matrix<double, 3, 3>::Constant(0) :
        (C_wv.transpose() * C_q.transpose() * state.get<StateDefinition_T::L>()(0)).eval(); //p_ci

    H.block<3, 3>(0, idxstartcorr_pvw_) = Eigen::Matrix<double, 3, 3>::Identity() * state.get<StateDefinition_T::L>()(0); //p_vw

    // attitude
    H.block<3, 3>(3, idxstartcorr_q_) = C_ci; // q

    H.block<3, 3>(3, idxstartcorr_qwv_) = driftvwattfix ? Eigen::Matrix<double, 3, 3>::Constant(0) : (C_ci * C_q).eval(); // q_wv

    H.block<3, 3>(3, idxstartcorr_qci_) = calibattfix ? Eigen::Matrix<double, 3, 3>::Constant(0) : Eigen::Matrix<double, 3, 3>::Identity().eval(); //q_ci

    //TODO: do we still want this?
    H(6, 18) = driftvwattfix ? 0 : 1.0; // fix vision world yaw drift because unobservable otherwise (see PhD Thesis)

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
      Eigen::Matrix<double, 3, 3> C_wv = state.get<StateDefinition_T::q_wv>().conjugate().toRotationMatrix();
      Eigen::Matrix<double, 3, 3> C_q = state.get<StateDefinition_T::q>().conjugate().toRotationMatrix();

      // construct residuals
      // position
      r_old.block<3, 1>(0, 0) = z_p_
          - (state.get<StateDefinition_T::p_vw>() + C_wv.transpose() *
              (state.get<StateDefinition_T::p>() + C_q.transpose() * state.get<StateDefinition_T::p_ci>()))
              * state.get<StateDefinition_T::L>();

      // attitude
      Eigen::Quaternion<double> q_err;
      q_err = (state.get<StateDefinition_T::q_wv>() * state.get<StateDefinition_T::q>() * state.get<StateDefinition_T::q_ci>()).conjugate() * z_q_;
      r_old.block<3, 1>(3, 0) = q_err.vec() / q_err.w() * 2;
      // vision world yaw drift
      q_err = state.get<StateDefinition_T::q_wv>();
      r_old(6, 0) = -2 * (q_err.w() * q_err.z() + q_err.x() * q_err.y()) / (1 - 2 * (q_err.y() * q_err.y() + q_err.z() * q_err.z()));

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

      // init variables
      //get previous measurement
      boost::shared_ptr<msf_core::MSF_MeasurementBase<EKFState_T> > prevmeas_base = core.getPreviousMeasurement(this->time, this->sensorID_);

      if(prevmeas_base->time == -1){
        ROS_WARN_STREAM("The previous measurement is invalid. Could not apply measurement! time:"<<this->time<<" sensorID: "<<this->sensorID_);
        return;
      }

      //try to make this a pose measurement
      boost::shared_ptr<PoseMeasurement> prevmeas = boost::shared_dynamic_cast<PoseMeasurement>(prevmeas_base);
      if(!prevmeas){
        ROS_WARN_STREAM("The dynamic cast of the previous measurement has failed. Could not apply measurement");
        return;
      }

      //get state at previous measurement
      boost::shared_ptr<EKFState_T> state_nonconst_old = core.getClosestState(prevmeas->time);

      if(state_nonconst_old->time == -1){
        ROS_WARN_STREAM("The state at the previous measurement is invalid. Could not apply measurement");
        return;
      }

      const EKFState_T& state_new = *state_nonconst_new; //get a const ref, so we can read core states
      const EKFState_T& state_old = *state_nonconst_old; //get a const ref, so we can read core states

      Eigen::Matrix<double, nMeasurements, msf_core::MSF_Core<EKFState_T>::nErrorStatesAtCompileTime> H_new, H_old;
      Eigen::Matrix<double, nMeasurements, 1> r_new, r_old;

      calculateH(state_nonconst_old, H_old);

      H_old *= -1;

      calculateH(state_nonconst_new, H_new);

      //TODO check that both measurements have the same states fixed!

      Eigen::Matrix<double, 3, 3> C_wv_old, C_wv_new;
      Eigen::Matrix<double, 3, 3> C_q_old, C_q_new;

      C_wv_new = state_new.get<StateDefinition_T::q_wv>().conjugate().toRotationMatrix();
      C_q_new = state_new.get<StateDefinition_T::q>().conjugate().toRotationMatrix();
      C_wv_old = state_old.get<StateDefinition_T::q_wv>().conjugate().toRotationMatrix();
      C_q_old = state_old.get<StateDefinition_T::q>().conjugate().toRotationMatrix();

      // construct residuals
      // position
      //TODO reenable p_vw
      Eigen::Matrix<double, 3, 1> diffprobpos = (/*state_new.get<StateDefinition_T::p_vw>() + */C_wv_new.transpose() * (state_new.get<StateDefinition_T::p>() + C_q_new.transpose() * state_new.get<StateDefinition_T::p_ci>()))
                                                     * state_new.get<StateDefinition_T::L>() -
                                                     (/*state_old.get<StateDefinition_T::p_vw>() + */C_wv_old.transpose() * (state_old.get<StateDefinition_T::p>() + C_q_old.transpose() * state_old.get<StateDefinition_T::p_ci>()))
                                                     * state_old.get<StateDefinition_T::L>();

      Eigen::Matrix<double, 3, 1> diffmeaspos = z_p_ - prevmeas->z_p_;

      r_new.block<3, 1>(0, 0) = diffmeaspos - diffprobpos;

      // attitude
      Eigen::Quaternion<double> diffprobatt = (state_new.get<StateDefinition_T::q_wv>() * state_new.get<StateDefinition_T::q>() * state_new.get<StateDefinition_T::q_ci>()).conjugate() * (state_old.get<StateDefinition_T::q_wv>() * state_old.get<StateDefinition_T::q>() * state_old.get<StateDefinition_T::q_ci>());
      Eigen::Quaternion<double> diffmeasatt = z_q_.conjugate() * prevmeas->z_q_;

      Eigen::Quaternion<double> q_err;
      q_err = diffprobatt.conjugate() * diffmeasatt;

      r_new.block<3, 1>(3, 0) = q_err.vec() / q_err.w() * 2;
      // vision world yaw drift
      q_err = state_new.get<StateDefinition_T::q_wv>();
      r_new(6, 0) = -2 * (q_err.w() * q_err.z() + q_err.x() * q_err.y()) / (1 - 2 * (q_err.y() * q_err.y() + q_err.z() * q_err.z()));


      if(!checkForNumeric(r_old, "r_old")){
        ROS_ERROR_STREAM("r_old: "<<r_old);
        ROS_WARN_STREAM("state: "<<const_cast<EKFState_T&>(state_new).toEigenVector().transpose());
      }
      if(!checkForNumeric(H_new, "H_old")){
        ROS_ERROR_STREAM("H_old: "<<H_new);
        ROS_WARN_STREAM("state: "<<const_cast<EKFState_T&>(state_new).toEigenVector().transpose());
      }
      if(!checkForNumeric(R_, "R_")){
        ROS_ERROR_STREAM("R_: "<<R_);
        ROS_WARN_STREAM("state: "<<const_cast<EKFState_T&>(state_new).toEigenVector().transpose());
      }

      // call update step in base class
      this->calculateAndApplyCorrectionRelative(state_nonconst_old, state_nonconst_new, core, H_old, H_new, r_new, R_);

    }
  }
};

}
}
#endif /* POSE_MEASUREMENT_HPP_ */
