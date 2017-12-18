/*
 * Copyright (C) 2012-2013 Simon Lynen, ASL, ETH Zurich, Switzerland
 * You can contact the author at <slynen at ethz dot ch>
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
#ifndef POSITION_POSE_SENSOR_MANAGER_H
#define POSITION_POSE_SENSOR_MANAGER_H

#include <ros/ros.h>

#include <msf_core/msf_core.h>
#include <msf_core/msf_sensormanagerROS.h>
#include <msf_core/msf_IMUHandler_ROS.h>
#include "msf_statedef.hpp"
#include <msf_updates/pose_sensor_handler/pose_sensorhandler.h>
#include <msf_updates/pose_sensor_handler/pose_measurement.h>
#include <msf_updates/position_sensor_handler/position_sensorhandler.h>
#include <msf_updates/position_sensor_handler/position_measurement.h>
#include <msf_updates/PositionPoseSensorConfig.h> //look at differences in these config files
//look very much the same


//dont I need these includes (as in pose sensor handler)
#include "sensor_fusion_comm/InitScale.h"
#include "sensor_fusion_comm/InitHeight.h"
//why is this namespace msf_updates when all other sensormanagers are in their respective msf_sensortype_sensor nampespace
namespace msf_updates {

typedef msf_updates::PositionPoseSensorConfig Config_T;
typedef dynamic_reconfigure::Server<Config_T> ReconfigureServer;
typedef shared_ptr<ReconfigureServer> ReconfigureServerPtr;

class PositionPoseSensorManager : public msf_core::MSF_SensorManagerROS<
    msf_updates::EKFState> {
  //this_T doesnt exits for pose handler but it simply refers to the class name
  typedef PositionPoseSensorManager this_T;
  typedef msf_pose_sensor::PoseSensorHandler<
      msf_updates::pose_measurement::PoseMeasurement<>, this_T> PoseSensorHandler_T;
  friend class msf_pose_sensor::PoseSensorHandler<
      msf_updates::pose_measurement::PoseMeasurement<>, this_T>;
  typedef msf_position_sensor::PositionSensorHandler<
      msf_updates::position_measurement::PositionMeasurement, this_T> PositionSensorHandler_T;
  friend class msf_position_sensor::PositionSensorHandler<
      msf_updates::position_measurement::PositionMeasurement, this_T>;
 //untit here seems to agree with pose_sensormanager and position sensor manager (may want to look at second 
//template argument of PoseMeasurement)
 public:
  typedef msf_updates::EKFState EKFState_T;
  typedef EKFState_T::StateSequence_T StateSequence_T;
  typedef EKFState_T::StateDefinition_T StateDefinition_T;

  PositionPoseSensorManager(
      ros::NodeHandle pnh = ros::NodeHandle("~/position_pose_sensor")) {

    bool distortmeas = false;  ///< Distort the pose measurements
    imu_handler_.reset(
        new msf_core::IMUHandler_ROS<msf_updates::EKFState>(*this, "msf_core",
                                                            "imu_handler"));

    pose_handler_.reset(
        new PoseSensorHandler_T(*this, "", "pose_sensor", distortmeas));
    AddHandler(pose_handler_);

    position_handler_.reset(
        new PositionSensorHandler_T(*this, "", "position_sensor"));
    AddHandler(position_handler_);

    reconf_server_.reset(new ReconfigureServer(pnh));
    ReconfigureServer::CallbackType f = boost::bind(&this_T::Config, this, _1,
                                                    _2);
    reconf_server_->setCallback(f);

    //dont we need this since we are using a pose sensor? (may not work exactly like this since different
    //class but could help)
    //no should be done somewhere else (where?)
    //init_scale_srv_ = pnh.advertiseService("initialize_msf_scale",
    //                                       &PoseSensorManager::InitScale, this);
    //init_height_srv_ = pnh.advertiseService("initialize_msf_height",
    //                                        &PoseSensorManager::InitHeight,
    //                                        this);
  }
  virtual ~PositionPoseSensorManager() {
      //shouldnt destructor actually do something since we are allocating other classes
      //like call their destructors (not necessary in other sensormanagers)
  }

  virtual void IncreaseNoise(int sensorID)
  {
    MSF_WARN_STREAM("NOT IMPLEMENTED FUNCTION");
    return;
}

  virtual const Config_T& Getcfg() {
    return config_;
  }

 private:
  shared_ptr<msf_core::IMUHandler_ROS<msf_updates::EKFState> > imu_handler_;
  shared_ptr<PoseSensorHandler_T> pose_handler_;
  shared_ptr<PositionSensorHandler_T> position_handler_;

  Config_T config_;
  ReconfigureServerPtr reconf_server_;  ///< Dynamic reconfigure server.
  //something with this init stuff that is in pose sensormanager seems to be missing here?
  ros::ServiceServer init_scale_srv_;
  ros::ServiceServer init_height_srv_;
  /**
   * \brief Dynamic reconfigure callback.
   */
  static constexpr double MIN_INITIALIZATION_HEIGHT = 0.01;
  virtual void Config(Config_T &config, uint32_t level) {
    config_ = config;
    pose_handler_->SetNoises(config.pose_noise_meas_p,
                             config.pose_noise_meas_q);
    pose_handler_->SetDelay(config.pose_delay);

    position_handler_->SetNoises(config.position_noise_meas);
    position_handler_->SetDelay(config.position_delay);

    if ((level & msf_updates::PositionPoseSensor_INIT_FILTER)
        && config.core_init_filter == true) {
      Init(config.pose_initial_scale); //initializes pose (how is this for position-> has always scale 1
    //and therefore need not be passed as argumen. still has to be initialized in Init)
      config.core_init_filter = false;
    }

    // Init call with "set height" checkbox.
    if ((level & msf_updates::PositionPoseSensor_SET_HEIGHT)
        && config.core_set_height == true) {
      Eigen::Matrix<double, 3, 1> p = pose_handler_->GetPositionMeasurement();
      if (p.norm() == 0) {
        MSF_WARN_STREAM(
            "No measurements received yet to initialize position. Height init "
            "not allowed.");
        return;
      }
      double scale = p[2] / config.core_height;
      Init(scale);
      config.core_set_height = false;
    }
}

//stuff from pose_sensormanager
bool InitScale(sensor_fusion_comm::InitScale::Request &req,
                 sensor_fusion_comm::InitScale::Response &res) {
    ROS_INFO("Initialize filter with scale %f", req.scale);
    Init(req.scale);
    res.result = "Initialized scale";
    return true;
  }

  bool InitHeight(sensor_fusion_comm::InitHeight::Request &req,
                  sensor_fusion_comm::InitHeight::Response &res) {
    ROS_INFO("Initialize filter with height %f", req.height);
    Eigen::Matrix<double, 3, 1> p = pose_handler_->GetPositionMeasurement();
    if (p.norm() == 0) {
      MSF_WARN_STREAM(
          "No measurements received yet to initialize position. Height init "
          "not allowed.");
      return false;
    }
    std::stringstream ss;
    if (std::abs(req.height) > MIN_INITIALIZATION_HEIGHT) {
      double scale = p[2] / req.height;
      Init(scale);
      ss << scale;
      res.result = "Initialized by known height. Initial scale = " + ss.str();
    } else {
      ss << "Height to small for initialization, the minimum is "
          << MIN_INITIALIZATION_HEIGHT << "and " << req.height << "was set.";
      MSF_WARN_STREAM(ss.str());
      res.result = ss.str();
      return false;
    }
    return true;
  }

  //to make it "cleaner" couldnt this call the functions of sensor memebers :thinking:
  void Init(double scale) const {
    if(!position_handler_->ReceivedFirstMeasurement() && !pose_handler_->ReceivedFirstMeasurement())
    {
        MSF_WARN_STREAM("No Measurements recieved at all. This hardly ever makes sense. Aborting Init");
        return;
    }
    //variables from pose
    Eigen::Matrix<double, 3, 1> p_ic, p_vc_c, p_wv;
    Eigen::Quaternion<double> q_wv, q_ic, q_cv;

    //variables from position
    Eigen::Matrix<double, 3, 1> p_ip, p_vc_p;

    //this SHOULD be the same for both since its orientation
    //if available get it from pose (since position has no orientation per se)
    //otherwise get it from yawinit (as in position sensor)
    //for position need to be somewhat smart. probably makes sense to use position sensor since its more "absolute"
    //than pose sensor (if available)
    Eigen::Quaternion<double> q;
    Eigen::Matrix<double, 3, 1> p;

    //variables that have same meaning+value in both
    Eigen::Matrix<double, 3, 1> v, b_w, b_a, g, w_m, a_m;
    msf_core::MSF_Core<EKFState_T>::ErrorStateCov P;

    // init values (why are they not taken from some settings file)
    //this means code needs to be recompiled (and wont speedup either since
    //not const)
    //these values are shared for both position and pose
    g << 0, 0, 9.81;	        /// Gravity.
    b_w << 0, 0, 0;		/// Bias gyroscopes.
    b_a << 0, 0, 0;		/// Bias accelerometer.

    v << 0, 0, 0;			/// Robot velocity (IMU centered).
    w_m << 0, 0, 0;		/// Initial angular velocity.

    P.setZero();  // Error state covariance; if zero, a default initialization in msf_core is used

    //variables only from pose
    q_wv.setIdentity();  // Vision-world rotation drift.
    p_wv.setZero();  // Vision-world position drift.

    p_vc_c = pose_handler_->GetPositionMeasurement(); //This is potentially different for both sensors but has same name
    q_cv = pose_handler_->GetAttitudeMeasurement();

    //variables only from position
    //probably only need yawinit if we cannot estimate it from pose->will see
    
    p_vc_p = position_handler_->GetPositionMeasurement(); //This is potentially different for both sensors but has same name
    

    // Check if we have already input from the measurement sensor.
    if (!pose_handler_->ReceivedFirstMeasurement())
    {
    MSF_WARN_STREAM(
        "No measurements received yet to initialize position - using [0 0 0]");
    MSF_WARN_STREAM(
        "No measurements received yet to initialize attitude - using [1 0 0 0]");
    }
    if (!position_handler_->ReceivedFirstMeasurement())
      MSF_WARN_STREAM(
          "No measurements received yet to initialize position - using [0 0 0]");
    ros::NodeHandle pnh("~");
    //from pose
    pnh.param("pose_sensor/init/p_ic/x", p_ic[0], 0.0);
    pnh.param("pose_sensor/init/p_ic/y", p_ic[1], 0.0);
    pnh.param("pose_sensor/init/p_ic/z", p_ic[2], 0.0);

    pnh.param("pose_sensor/init/q_ic/w", q_ic.w(), 1.0);
    pnh.param("pose_sensor/init/q_ic/x", q_ic.x(), 0.0);
    pnh.param("pose_sensor/init/q_ic/y", q_ic.y(), 0.0);
    pnh.param("pose_sensor/init/q_ic/z", q_ic.z(), 0.0);
    q_ic.normalize();
    //from position
    pnh.param("position_sensor/init/p_ip/x", p_ip[0], 0.0);
    pnh.param("position_sensor/init/p_ip/y", p_ip[1], 0.0);
    pnh.param("position_sensor/init/p_ip/z", p_ip[2], 0.0);

    // Calculate initial attitude and position based on sensor measurements.
    if (!pose_handler_->ReceivedFirstMeasurement()) {  // If there is no pose measurement, compute q as in position sensormanager
      double yawinit = config_.position_yaw_init / 180 * M_PI;
      Eigen::Quaterniond yawq(cos(yawinit / 2), 0, 0, sin(yawinit / 2));
      yawq.normalize();
      q = yawq; 
    }
    else if(!position_handler_->ReceivedFirstMeasurement())//if there is no position measurement compute q as in pose sensormanager
    {
        q = (q_ic * q_cv.conjugate() * q_wv).conjugate(); //i believe quaternions here are handled as matrices
        q.normalize();
    }
    else {  // If there are both take orientation from position handler (since we want to live in position frame)
      double yawinit = config_.position_yaw_init / 180 * M_PI;
      Eigen::Quaterniond yawq(cos(yawinit / 2), 0, 0, sin(yawinit / 2));
      yawq.normalize();
      q = yawq; 
      //compute new transform for pose (rotational part)
      //Eigen::Quaternion<double> temp = (q_ic * q_cv.conjugate() * q_wv).conjugate();
      //q_ic = (q.toRotationMatrix().inverse()*temp.toRotationMatrix());
      q_ic = q.conjugate()*q_wv.inverse()*q_cv.conjugate().inverse(); //better but still not correct
    }

    
    //need to think carefully about next part: want to have one "6Dof pose" for both sensors togethe
    //but might potentially be different:
    //easy case: only one recieved measurement->need to ignore second
    //otherwise may somehow "average"
    //also should probably compute transform between the 2 somehow based on that
    //will somehow have to compute transform between pos and pose (given transformation is only from measurement point
    //to IMU but coordinate frame is potentially very different)
    //maybe smartest is to go to position frame? (since this is somewhat absolute)
    //basically: if there is only one measurement given we simply take this as "worldframe"
    //(after transformation according to parameters)
    //if both are given we need to decide one (should prefare position probably)
    //and correct the transformation of the second one
    //this will also be important for delayed initialization and so on (initializing only one sensor->how to?)

    //only position measurement recieved. position as in position sensormanager
    if(!pose_handler_->ReceivedFirstMeasurement())
    {
        p = p_vc_p - q.toRotationMatrix() * p_ip;
    }
    //only pose measurement recieved. position as in pose sensormanager
    else if(!position_handler_->ReceivedFirstMeasurement())
    {
        p = p_wv + q_wv.conjugate().toRotationMatrix() * p_vc_c / scale
        - q.toRotationMatrix() * p_ic;
    }
    
    //both measurements recieved...need to do something smart 
    else
    {
        //take position world frame
        p = p_vc_p - q.toRotationMatrix() * p_ip;
        //adjust pose transformation
        p_ic=q_ic.toRotationMatrix().inverse()*(q_wv.conjugate().toRotationMatrix() * p_vc_c / scale - p);
    }


    a_m = q.inverse() * g;			/// Initial acceleration.
    // Prepare init "measurement"
    // True means that this message contains initial sensor readings.
    shared_ptr < msf_core::MSF_InitMeasurement<EKFState_T>
        > meas(new msf_core::MSF_InitMeasurement<EKFState_T>(true));

    meas->SetStateInitValue < StateDefinition_T::p > (p);
    meas->SetStateInitValue < StateDefinition_T::v > (v);
    meas->SetStateInitValue < StateDefinition_T::q > (q);
    meas->SetStateInitValue < StateDefinition_T::b_w > (b_w);
    meas->SetStateInitValue < StateDefinition_T::b_a > (b_a);
    meas->SetStateInitValue < StateDefinition_T::L
        > (Eigen::Matrix<double, 1, 1>::Constant(scale));
    meas->SetStateInitValue < StateDefinition_T::q_wv > (q_wv);
    meas->SetStateInitValue < StateDefinition_T::p_wv > (p_wv);
    meas->SetStateInitValue < StateDefinition_T::q_ic > (q_ic);
    meas->SetStateInitValue < StateDefinition_T::p_ic > (p_ic);
    meas->SetStateInitValue < StateDefinition_T::p_ip > (p_ip);

    SetStateCovariance(meas->GetStateCovariance());  // Call my set P function.
    meas->Getw_m() = w_m;
    meas->Geta_m() = a_m;
    meas->time = ros::Time::now().toSec();

    // Call initialization in core.
    msf_core_->Init(meas);
    
}


  /*void Init(double scale) const {
    Eigen::Matrix<double, 3, 1> p, v, b_w, b_a, g, w_m, a_m, p_ic, p_vc, p_wv;
    Eigen::Quaternion<double> q, q_wv, q_ic, q_cv;
    msf_core::MSF_Core<EKFState_T>::ErrorStateCov P;

    // init values
    g << 0, 0, 9.81;	        /// Gravity.
    b_w << 0, 0, 0;		/// Bias gyroscopes.
    b_a << 0, 0, 0;		/// Bias accelerometer.

    v << 0, 0, 0;			/// Robot velocity (IMU centered).
    w_m << 0, 0, 0;		/// Initial angular velocity.

    q_wv.setIdentity();  // Vision-world rotation drift.
    p_wv.setZero();  // Vision-world position drift.

    P.setZero();  // Error state covariance; if zero, a default initialization in msf_core is used

    p_vc = pose_handler_->GetPositionMeasurement();
    q_cv = pose_handler_->GetAttitudeMeasurement();

    MSF_INFO_STREAM(
        "initial measurement pos:["<<p_vc.transpose()<<"] orientation: "<<STREAMQUAT(q_cv));
    MSF_WARN_STREAM("We are in Init function");
    if (!pose_handler_->ReceivedFirstMeasurement())
    {
        MSF_WARN_STREAM(
          "Pose sensor did not recieve any measurement yet");
        }
    if (!position_handler_->ReceivedFirstMeasurement())
    {
        MSF_WARN_STREAM(
          "Position sensor did not recieve any measurement yet");
    }
    // Check if we have already input from the measurement sensor.
    if (p_vc.norm() == 0)
      MSF_WARN_STREAM(
          "No measurements received yet to initialize position - using [0 0 0]");
    if (q_cv.w() == 1)
      MSF_WARN_STREAM(
          "No measurements received yet to initialize attitude - using [1 0 0 0]");

    ros::NodeHandle pnh("~");
    pnh.param("pose_sensor/init/p_ic/x", p_ic[0], 0.0);
    pnh.param("pose_sensor/init/p_ic/y", p_ic[1], 0.0);
    pnh.param("pose_sensor/init/p_ic/z", p_ic[2], 0.0);

    pnh.param("pose_sensor/init/q_ic/w", q_ic.w(), 1.0);
    pnh.param("pose_sensor/init/q_ic/x", q_ic.x(), 0.0);
    pnh.param("pose_sensor/init/q_ic/y", q_ic.y(), 0.0);
    pnh.param("pose_sensor/init/q_ic/z", q_ic.z(), 0.0);
    q_ic.normalize();

    // Calculate initial attitude and position based on sensor measurements.
    if (!pose_handler_->ReceivedFirstMeasurement()) {  // If there is no pose measurement, only apply q_wv.
      q = q_wv;
    } else {  // If there is a pose measurement, apply q_ic and q_wv to get initial attitude.
      q = (q_ic * q_cv.conjugate() * q_wv).conjugate();
    }

    q.normalize();
    p = p_wv + q_wv.conjugate().toRotationMatrix() * p_vc / scale
        - q.toRotationMatrix() * p_ic;

    a_m = q.inverse() * g;			/// Initial acceleration.

    // Prepare init "measurement"
    // True means that this message contains initial sensor readings.
    shared_ptr < msf_core::MSF_InitMeasurement<EKFState_T>
        > meas(new msf_core::MSF_InitMeasurement<EKFState_T>(true));

    meas->SetStateInitValue < StateDefinition_T::p > (p);
    meas->SetStateInitValue < StateDefinition_T::v > (v);
    meas->SetStateInitValue < StateDefinition_T::q > (q);
    meas->SetStateInitValue < StateDefinition_T::b_w > (b_w);
    meas->SetStateInitValue < StateDefinition_T::b_a > (b_a);
    meas->SetStateInitValue < StateDefinition_T::L
        > (Eigen::Matrix<double, 1, 1>::Constant(scale));
    meas->SetStateInitValue < StateDefinition_T::q_wv > (q_wv);
    meas->SetStateInitValue < StateDefinition_T::p_wv > (p_wv);
    meas->SetStateInitValue < StateDefinition_T::q_ic > (q_ic);
    meas->SetStateInitValue < StateDefinition_T::p_ic > (p_ic);

    SetStateCovariance(meas->GetStateCovariance());  // Call my set P function.
    meas->Getw_m() = w_m;
    meas->Geta_m() = a_m;
    meas->time = ros::Time::now().toSec();

    // Call initialization in core.
    msf_core_->Init(meas);

  }*/

  // Prior to this call, all states are initialized to zero/identity.
  virtual void ResetState(EKFState_T& state) const {
    //set scale to 1
    Eigen::Matrix<double, 1, 1> scale;
    scale << 1.0;
    state.Set < StateDefinition_T::L > (scale);
  }
  virtual void InitState(EKFState_T& state) const {
    UNUSED(state);
  }

  virtual void CalculateQAuxiliaryStates(EKFState_T& state, double dt) const {
    const msf_core::Vector3 nqwvv = msf_core::Vector3::Constant(
        config_.pose_noise_q_wv);
    const msf_core::Vector3 npwvv = msf_core::Vector3::Constant(
        config_.pose_noise_p_wv);
    const msf_core::Vector3 nqicv = msf_core::Vector3::Constant(
        config_.pose_noise_q_ic);
    const msf_core::Vector3 npicv = msf_core::Vector3::Constant(
        config_.pose_noise_p_ic);
    const msf_core::Vector1 n_L = msf_core::Vector1::Constant(
        config_.pose_noise_scale);

    // Compute the blockwise Q values and store them with the states,
    // these then get copied by the core to the correct places in Qd.
    state.GetQBlock<StateDefinition_T::L>() = (dt * n_L.cwiseProduct(n_L))
        .asDiagonal();
    state.GetQBlock<StateDefinition_T::q_wv>() =
        (dt * nqwvv.cwiseProduct(nqwvv)).asDiagonal();
    state.GetQBlock<StateDefinition_T::p_wv>() =
        (dt * npwvv.cwiseProduct(npwvv)).asDiagonal();
    state.GetQBlock<StateDefinition_T::q_ic>() =
        (dt * nqicv.cwiseProduct(nqicv)).asDiagonal();
    state.GetQBlock<StateDefinition_T::p_ic>() =
        (dt * npicv.cwiseProduct(npicv)).asDiagonal();
  }

  virtual void SetStateCovariance(
      Eigen::Matrix<double, EKFState_T::nErrorStatesAtCompileTime,
          EKFState_T::nErrorStatesAtCompileTime>& P) const {
    UNUSED(P);
    // Nothing, we only use the simulated cov for the core plus diagonal for the
    // rest.
  }

  virtual void AugmentCorrectionVector(
      Eigen::Matrix<double, EKFState_T::nErrorStatesAtCompileTime, 1>& correction) const {
    UNUSED(correction);
  }

  virtual void SanityCheckCorrection(
      EKFState_T& delaystate,
      const EKFState_T& buffstate,
      Eigen::Matrix<double, EKFState_T::nErrorStatesAtCompileTime, 1>& correction) const {
    UNUSED(buffstate);
    UNUSED(correction);

    const EKFState_T& state = delaystate;
    if (state.Get<StateDefinition_T::L>()(0) < 0) {
      MSF_WARN_STREAM_THROTTLE(
          1,
          "Negative scale detected: " << state.Get<StateDefinition_T::L>()(0) << ". Correcting to 0.1");
      Eigen::Matrix<double, 1, 1> L_;
      L_ << 0.1;
      delaystate.Set < StateDefinition_T::L > (L_);
    }
  }
};
}
#endif  // POSITION_POSE_SENSOR_MANAGER_H
