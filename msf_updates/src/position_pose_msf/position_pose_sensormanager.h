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
#include <msf_updates/PositionPoseSensorConfig.h> 

#include <Eigen/Geometry>

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
    pnh.param("use_stable_initialization", use_stable_initialization_, false);
    if(use_stable_initialization_)
    {
        MSF_WARN_STREAM("sensormanager is using stable initialization");
    }
    //initializing sensorID with 0
    ResetSensorID();
    //now each sensor will have a ID according to its position in vector (which can be used to access)
    imu_handler_.reset(
        new msf_core::IMUHandler_ROS<msf_updates::EKFState>(*this, "msf_core",
                                                            "imu_handler"));

    pose_handler_.reset(
        new PoseSensorHandler_T(*this, "", "pose_sensor", distortmeas));
    AddHandler(pose_handler_); //this will be ID=0

    position_handler_.reset(
        new PositionSensorHandler_T(*this, "", "position_sensor"));
    AddHandler(position_handler_); //this will be ID=1

    reconf_server_.reset(new ReconfigureServer(pnh));
    ReconfigureServer::CallbackType f = boost::bind(&this_T::Config, this, _1,
                                                    _2);
    reconf_server_->setCallback(f);

  }
  virtual ~PositionPoseSensorManager() {
      //shouldnt destructor actually do something since we are allocating other classes
      //like call their destructors (not necessary in other sensormanagers)
  }

  virtual const Config_T& Getcfg() {
    return config_;
  }

 private:
  shared_ptr<msf_core::IMUHandler_ROS<msf_updates::EKFState> > imu_handler_;
  shared_ptr<PoseSensorHandler_T> pose_handler_;
  shared_ptr<PositionSensorHandler_T> position_handler_;

  Config_T config_; //this is really unlucky to have the same name as parents config_......
  ReconfigureServerPtr reconf_server_;  ///< Dynamic reconfigure server.

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
  void Init(double scale){
    if(!position_handler_->ReceivedFirstMeasurement() && !pose_handler_->ReceivedFirstMeasurement())
    {
        MSF_WARN_STREAM("No Measurements recieved at all. This hardly ever makes sense. Aborting Init");
        return;
    }
    if(position_handler_->use_nn_noise_estimation_)
    {
        //if a sensor is using nn noise estimation make service call to tf evaluation node
        ros::NodeHandle ntemp;
        ros::ServiceClient clienttemp = ntemp.serviceClient<sensor_fusion_comm::AddListener>("eval_node/add_listener");
        sensor_fusion_comm::AddListener srvtemp;
        srvtemp.request.key = position_handler_->tf_key_;
        srvtemp.request.tfnetworkpath = position_handler_->tf_network_path_;
        srvtemp.request.tfnetworkweights = position_handler_->tf_network_weights_;
        srvtemp.request.evalfrequency = position_handler_->tf_eval_frequency_;
        srvtemp.request.maxmemory = position_handler_->tf_max_memory_;

        clienttemp.call(srvtemp);
    }
    if(pose_handler_->use_nn_noise_estimation_)
    {
        //if a sensor is using nn noise estimation make service call to tf evaluation node
        ros::NodeHandle ntemp;
        ros::ServiceClient clienttemp = ntemp.serviceClient<sensor_fusion_comm::AddListener>("eval_node/add_listener");
        sensor_fusion_comm::AddListener srvtemp;
        srvtemp.request.key = pose_handler_->tf_key_;
        srvtemp.request.tfnetworkpath = pose_handler_->tf_network_path_;
        srvtemp.request.tfnetworkweights = pose_handler_->tf_network_weights_;
        srvtemp.request.evalfrequency = pose_handler_->tf_eval_frequency_;
        srvtemp.request.maxmemory = pose_handler_->tf_max_memory_;

        clienttemp.call(srvtemp);
    }
    if(use_stable_initialization_)
    {
        //tell sensorhandlers to start collecting for stable initialization
        MSF_INFO_STREAM("Collecting for stable initialization started");
        pose_handler_->collect_for_init_=true;
        pose_handler_->init_points_.clear();
        pose_handler_->total_init_movement_=0.0;
        position_handler_->collect_for_init_=true;
        position_handler_->init_points_.clear();
        position_handler_->total_init_movement_=0.0;
        //don't need to do anything else rn: wait until both sensors tell ready
    }
    else
    {
        //variables from pose
        Eigen::Matrix<double, 3, 1> p_ic, p_vc_c, p_wv;
        Eigen::Quaternion<double> q_wv, q_ic, q_cv;
        Eigen::Quaterniond initpose_vision;

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

        Eigen::Quaternion<double> initpose_gps;
        pnh.param("position_sensor/init/pose/w", initpose_gps.w(), 1.0);
        pnh.param("position_sensor/init/pose/x", initpose_gps.x(), 0.0);
        pnh.param("position_sensor/init/pose/y", initpose_gps.y(), 0.0);
        pnh.param("position_sensor/init/pose/z", initpose_gps.z(), 0.0);

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
            //this is garbage

            q_ic.normalize();
            q_cv.normalize();
            //orientation of the drone in GPS frame (for now assume given)
            //Eigen::Quaterniond initpose_gps(0.993240709, -0.0092359533, 0.0225063474, 0.1134947378); //at 0 secs V1_easy (w,x,y,z)
            //Eigen::Quaterniond initpose_gps(0.997609002734, -0.0191316931928, 0.0233401735257, -0.062173083234); //at 0 secs V1_hard (w,x,y,z)
            if (initpose_gps.w()==1.0)
            {
                //not set use yawinit (probably not precise, should use stable init instead)
                double yawinit = config_.position_yaw_init / 180 * M_PI;
                Eigen::Quaterniond yawq(cos(yawinit / 2), 0, 0, sin(yawinit / 2));
                yawq.normalize();
                initpose_gps = yawq; 
            }
            initpose_gps.normalize();
            //orientation of the drone in Vision frame
            
            initpose_vision=q_cv*q_ic.conjugate();
            initpose_vision.normalize();

            q_wv=initpose_vision*initpose_gps.conjugate();
            q=initpose_gps;

        }

        
        //do same here with transformation (i.e. think carefully)
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
        
        //both measurements recieved
        //based on current measurement and assumption that orientation in position frame is given
        //compute transform p_wv, q_wv such that both sensors map to same pose in common frame
        else
        {
            //position of IMU in GPS frame THINK ABOUT THIS
            p = p_vc_p - q.toRotationMatrix() * p_ip; //is - right here
            //position of IMU in Vision frame
            Eigen::Matrix<double, 3, 1> p_vision;
            p_vision = (p_vc_c/scale - initpose_vision.toRotationMatrix()*p_ic); //same here with -
            p_wv=p-q_wv.conjugate()*p_vision;
            
        }

        if(pose_handler_->use_transform_recovery_)
        {
        config_.pose_noise_p_wv = std::min(config_.pose_noise_meas_p/2.0, pose_handler_->transform_recovery_noise_p_);
        config_.pose_noise_q_wv = std::min(config_.pose_noise_meas_q/2.0, pose_handler_->transform_recovery_noise_q_);
        pose_handler_->transform_curr_anealing_steps_ = pose_handler_->transform_anealing_steps_-2;
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
    
}

void InitStable()
{
    double scale=1.0; //should do something with that
    //check wether all sensors are ready for init
    if(!pose_handler_->ready_for_init_ || !position_handler_->ready_for_init_)
    {
        //not ready yet
        return;
    }

    //the difference we allow (between measurements)
    const double epsilon = 1e-6;
    //both will be resized once we know how many entries to expect
    Eigen::MatrixXd positionframe(1,1);
    Eigen::MatrixXd poseframe(1,1);
    
    //find first measurement that is ~at same time
    //iterators would be nicer but need to walk backward aswell
    int positioncurr=0;
    int posecurr=0;
    int positionfinal=position_handler_->init_points_.size()-1;
    int posefinal=pose_handler_->init_points_.size()-1;
    if(position_handler_->init_points_[positioncurr](3)>pose_handler_->init_points_[posecurr](3))
    {
        while(position_handler_->init_points_[positioncurr](3)>pose_handler_->init_points_[posecurr](3)+epsilon)
        {
            posecurr++;
        }
    }
    else
    {
        while(pose_handler_->init_points_[posecurr](3)>position_handler_->init_points_[positioncurr](3)+epsilon)
        {
            positioncurr++;
        }
    }
    //do same for end
    if(position_handler_->init_points_[positionfinal](3)>pose_handler_->init_points_[posefinal](3))
    {
        while(position_handler_->init_points_[positionfinal](3)>pose_handler_->init_points_[posefinal](3)+epsilon)
        {
            positionfinal--;
        }
    }
    else
    {
        while(pose_handler_->init_points_[posefinal](3)>position_handler_->init_points_[positionfinal](3)+epsilon)
        {
            posefinal--;
        }
    }

    //now both iterators should be at measurement with approx same time
    //check which vector is shorter (this determines next meas)
    //do not locally transform pose (since we dont know complete rotation)
    //can reverse engineer it later
    if((positionfinal-positioncurr)>(posefinal-posecurr))
    {
        //compute number of entries: (not sure this works like that)
        const int nentries = posefinal-posecurr-2; //-2 since we want to ignore the last entry (could potentially have to big time diff) and we average 2
        positionframe.resize(3,nentries);
        poseframe.resize(3,nentries);
        for(int i=0;i<nentries;++i)
        {
            if(posecurr+1>=posefinal)
            {
                MSF_WARN_STREAM("pose iterator overshooting...this should not be possible");
            }
            //MSF_WARN_STREAM("iteration:"<<i);
            Eigen::Vector3d temppose = (pose_handler_->init_points_[posecurr].head(3)+pose_handler_->init_points_[++posecurr].head(3))/2.0;
            int ntemp=1;
            Eigen::Vector3d temppos = position_handler_->init_points_[positioncurr].head(3); //dont transform since we dont know orientation
            positioncurr++;
            while(position_handler_->init_points_[positioncurr](3)+epsilon<pose_handler_->init_points_[posecurr](3))
            {
                temppos+=position_handler_->init_points_[positioncurr].head(3);
                ntemp++;
                positioncurr++;
                if(positioncurr>=positionfinal)
                {
                    MSF_WARN_STREAM("position iterator overshooting...this is strange but not impossible");
                }
            }
            poseframe.col(i)=temppose;
            positionframe.col(i)=temppos/ntemp;
        }
    }
    //do not locally transform pose (since we dont know complete rotation)
    //can reverse engineer it later
    else
    {
        //compute number of entries: (not sure this works like that)
        const int nentries = positionfinal-positioncurr-2;
        positionframe.resize(3,nentries);
        poseframe.resize(3,nentries);
        for(int i=0;i<nentries;++i)
        {
            Eigen::Vector3d temppos = (position_handler_->init_points_[positioncurr].head(3)+position_handler_->init_points_[++positioncurr].head(3))/2.0; //dont transform since we dont know orientation
            int ntemp=1;
            Eigen::Vector3d temppose = pose_handler_->init_points_[posecurr].head(3);
            posecurr++;
            while(pose_handler_->init_points_[posecurr](3)+epsilon<position_handler_->init_points_[positioncurr](3))
            {
                temppose+=pose_handler_->init_points_[posecurr].head(3);
                ntemp++;
                posecurr++;
            }
            poseframe.col(i)=temppose/ntemp;
            positionframe.col(i)=temppos;
        }
    }

    //proper way of checking for stability would be: COV(X,Y)=1/nYKX^T, with K={1-1/n on diag and -1/n else} has rank>=d-1=2 (now should check that first 2 eigenvalues are "large")
    //try this (need to set 1 handler ready_for_init false, else this won't be called again, need to do time alignement first)
    const int npoints = positionframe.cols();
    Eigen::MatrixXd K = Eigen::MatrixXd::Constant(npoints, npoints, -1.0/npoints);
    K.diagonal() = Eigen::ArrayXd::Constant(npoints, 1.0-1.0/npoints);
    Eigen::MatrixXd covariance = 1.0/npoints*positionframe*K*poseframe.transpose();
    Eigen::VectorXd eigenvalues = covariance.eigenvalues().real().cwiseAbs();
    std::sort(eigenvalues.data(), eigenvalues.data()+eigenvalues.size(), std::greater<double>());
    //now should be sorted
    //now the first two eigenvalues should be >= some value for stability (its sufficient to look at 2nd largest)
    if(eigenvalues(1)<0.02) //try this 
    {
      //not stable dont initialize
      //set one to false s.t. this gets called again
      pose_handler_->ready_for_init_=false;
      return;
    }
    MSF_INFO_STREAM("initstable called and all sensors ready, using Eigen::umeyama to estimate transform");
    position_handler_->ready_for_init_=false;
    position_handler_->collect_for_init_=false;
    pose_handler_->ready_for_init_=false;
    pose_handler_->collect_for_init_=false;
    


    //call eigen umeyama (gives transform from ROVIO frame to GPS frame)
    Eigen::MatrixXd homogeneous_transform = Eigen::umeyama(poseframe, positionframe, false); //not 100% about from to (think its correct)
    //postprocess transforms and do actual init using transformed estimate from pose (since its expected to have less noise)
    Eigen::Matrix3d R_temp = homogeneous_transform.topLeftCorner(3,3);
    Eigen::Quaterniond q_pose(R_temp);
    Eigen::Vector3d p_pose = homogeneous_transform.topRightCorner(3,1);

    //normalize just to make sure
    q_pose.normalize();


    //variables from pose
    Eigen::Matrix<double, 3, 1> p_ic, p_vc_c, p_wv;
    Eigen::Quaternion<double> q_wv, q_ic, q_cv;

    //variables from position
    Eigen::Matrix<double, 3, 1> p_ip, p_vc_p;

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

    v << 0, 0, 0;			/// Robot velocity (IMU centered). why zero? :thinking: if this is large in reality will cause relativly large error?
    //we could estimate it based on last few rovio data i think
    w_m << 0, 0, 0;		/// Initial angular velocity.

    P.setZero();  // Error state covariance; if zero, a default initialization in msf_core is used

    //variables only from pose
    //set those as transform from vision world to gps world (estimated from points)
    q_wv.setIdentity();  // Vision-world rotation drift.
    p_wv.setZero();  // Vision-world position drift.

    p_vc_c = pose_handler_->GetPositionMeasurement(); 
    q_cv = pose_handler_->GetAttitudeMeasurement();

    //variables only from position
    //probably only need yawinit if we cannot estimate it from pose->will see
    
    p_vc_p = position_handler_->GetPositionMeasurement(); 
    

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


    //Eigen::Quaternion<double> q_imu = (q_ic * q_cv.conjugate()).conjugate(); //this should be drone orientation in its own frame
    //this should be the same as q_cv*q_ic.conjugate()

    q_wv=q_pose.conjugate();
    //q_wv=q_pose;
    q_wv.normalize();
    //q = (q_ic * q_cv.conjugate() * q_wv).conjugate();
    q=q_wv.conjugate()*q_cv*q_ic.conjugate();
    q.normalize();




    //p = q_pose*p_vc_c + p_pose-q*p_ic; //i think this is correct
    p=p_vc_p-q*p_ip;
    //from this we can compute p_wv (not sure about this one)
    p_wv = p - q_wv.conjugate().toRotationMatrix() * (p_vc_c / scale +  q_cv*q_ic.conjugate()* p_ic); //this should give the same as p_wv=p_pose

    if(pose_handler_->use_transform_recovery_)
    {
    config_.pose_noise_p_wv = pose_handler_->transform_recovery_noise_p_;
    config_.pose_noise_q_wv = pose_handler_->transform_recovery_noise_q_;
    pose_handler_->transform_curr_anealing_steps_ = pose_handler_->transform_anealing_steps_;
    }
    a_m = q.inverse() * g;			/// Initial acceleration. (i think this is garbage)
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

  //distinguish between sensors and reinit this sensor only
  //be careful about transformations
  void Initsingle(int sensorID) const
  {
    //this is not working yet-> how to get state look at line 421 and GetStateVariable
    //ID==0 is pose sensor
    if(sensorID==0)
    {
        const shared_ptr<EKFState_T>& latestState = msf_core_->GetLastState();
        //this is how to do this
        //const Eigen::Matrix<double,3,1> temp=const_cast<const EKFState_T&>(*latestState).template Get<StateDefinition_T::v>();
        //init pose sensor
        if(!pose_handler_->ReceivedFirstMeasurement())
        {
            MSF_WARN_STREAM("Cannot Reinitialize pose sensor without measurements. Aborting");
            return;
        }
        //we do not change scale when reinitializing one sensor->get scale from last state
        const double scale = const_cast<const EKFState_T&>(*latestState).template Get<StateDefinition_T::L>()(0,0); //for some reasone scale is saved as a 1 by 1 matrix... (might want to change this eventually)
        //variables from pose
        Eigen::Matrix<double, 3, 1> p_wv, p_vc_c;
        Eigen::Quaternion<double> q_wv, q_cv;

        const Eigen::Matrix<double, 3, 1> p_ip = const_cast<const EKFState_T&>(*latestState).template Get<StateDefinition_T::p_ip>();

        //since we are only reinitializing one sensor take last state as "truth" and recompute this transform
        const Eigen::Quaternion<double> q = const_cast<const EKFState_T&>(*latestState).template Get<StateDefinition_T::q>();;
        const Eigen::Matrix<double, 3, 1> p = const_cast<const EKFState_T&>(*latestState).template Get<StateDefinition_T::p>();;

        //variables that have same meaning+value in both
        Eigen::Matrix<double, 3, 1> g, w_m, a_m;
        msf_core::MSF_Core<EKFState_T>::ErrorStateCov P;

        //here the values have to be the same as in latest state
        //these values are shared for both position and pose
        g << 0, 0, 9.81;	        /// Gravity.
        w_m << 0, 0, 0;             //angular velocity (how to access this from state)

        const Eigen::Matrix<double, 3, 1> b_w = const_cast<const EKFState_T&>(*latestState).template Get<StateDefinition_T::b_w>();
        const Eigen::Matrix<double, 3, 1> b_a = const_cast<const EKFState_T&>(*latestState).template Get<StateDefinition_T::b_a>();
        const Eigen::Matrix<double, 3, 1> v = const_cast<const EKFState_T&>(*latestState).template Get<StateDefinition_T::v>();		/// Robot velocity (IMU centered).
        
        //const Eigen::Matrix<double, 3, 1> w_m = const_cast<const EKFState_T&>(*latestState).template Get<StateDefinition_T::w_m>();		/// to access this is different (how/why?)

        //idk about that one yet
        P.setZero();  // Error state covariance; if zero, a default initialization in msf_core is used

        //variables only from pose
        const Eigen::Quaternion<double> q_ic = const_cast<const EKFState_T&>(*latestState).template Get<StateDefinition_T::q_ic>();
        const Eigen::Matrix<double, 3, 1> p_ic = const_cast<const EKFState_T&>(*latestState).template Get<StateDefinition_T::p_ic>();

        p_vc_c = pose_handler_->GetPositionMeasurement(); //This is potentially different for both sensors but has same name
        q_cv = pose_handler_->GetAttitudeMeasurement();

        //position is given by last state
        //we do not touch position sensor so we need to recompute transformation of pose sensor
        //compute new transform for pose (rotational part)
        //q_ic = q.conjugate()*q_wv.inverse()*q_cv.conjugate().inverse(); //looks good now
        q_wv = q_cv*q_ic.conjugate()*q.conjugate();
        q_wv.normalize();
        //q_ic.normalize();

        //compute new tranfor for pose (translational part)
        //p_ic=q_ic.toRotationMatrix().inverse()*(p_wv + q_wv.conjugate().toRotationMatrix() * p_vc_c / scale - p);
        //MSF_WARN_STREAM("new p_ic"<<p_ic<<"p_wv:"<<p_wv<<"p_vc_c:"<<p_vc_c<<"scale"<<scale<<"p"<<p);
        p_wv = p - q_wv.conjugate().toRotationMatrix() * p_vc_c / scale + q.toRotationMatrix() * p_ic;

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
    //ID==1 is position sensor
    else if(sensorID==1)
    {
         const shared_ptr<EKFState_T>& latestState = msf_core_->GetLastState();
        //this is how to do this
        //const Eigen::Matrix<double,3,1> temp=const_cast<const EKFState_T&>(*latestState).template Get<StateDefinition_T::v>();
        //init position sensor
        if(!position_handler_->ReceivedFirstMeasurement())
        {
            MSF_WARN_STREAM("Cannot Reinitialize position sensor without measurements. Aborting");
            return;
        }
        //we do not change scale when reinitializing one sensor->get scale from last state
        const double scale = const_cast<const EKFState_T&>(*latestState).template Get<StateDefinition_T::L>()(0,0); //for some reasone scale is saved as a 1 by 1 matrix... (might want to change this eventually)
        //variables from position
        Eigen::Matrix<double, 3, 1> p_ip, p_vc_p;

        const Eigen::Matrix<double, 3, 1> p_ic = const_cast<const EKFState_T&>(*latestState).template Get<StateDefinition_T::p_ic>();
        const Eigen::Quaternion<double> q_ic = const_cast<const EKFState_T&>(*latestState).template Get<StateDefinition_T::q_ic>();

        //since we are only reinitializing one sensor take last state as "truth" and recompute this transform
        const Eigen::Quaternion<double> q = const_cast<const EKFState_T&>(*latestState).template Get<StateDefinition_T::q>();
        const Eigen::Matrix<double, 3, 1> p = const_cast<const EKFState_T&>(*latestState).template Get<StateDefinition_T::p>();

        //variables that have same meaning+value in both
        Eigen::Matrix<double, 3, 1> g, w_m, a_m;
        msf_core::MSF_Core<EKFState_T>::ErrorStateCov P;

        //here the values have to be the same as in latest state
        //these values are shared for both position and pose
        g << 0, 0, 9.81;	        /// Gravity.
        w_m << 0, 0, 0;             //angular velocity (how to access this from state)
        
        const Eigen::Matrix<double, 3, 1> b_w = const_cast<const EKFState_T&>(*latestState).template Get<StateDefinition_T::b_w>();
        const Eigen::Matrix<double, 3, 1> b_a = const_cast<const EKFState_T&>(*latestState).template Get<StateDefinition_T::b_a>();
        const Eigen::Matrix<double, 3, 1> v = const_cast<const EKFState_T&>(*latestState).template Get<StateDefinition_T::v>();		/// Robot velocity (IMU centered).
        
        //const Eigen::Matrix<double, 3, 1> w_m = const_cast<const EKFState_T&>(*latestState).template Get<StateDefinition_T::w_m>();		/// to access this is different (how/why?)

        //idk about that one yet
        P.setZero();  // Error state covariance; if zero, a default initialization in msf_core is used

        //variables only from pose
        const Eigen::Quaternion<double> q_wv = const_cast<const EKFState_T&>(*latestState).template Get<StateDefinition_T::q_wv>();
        const Eigen::Matrix<double, 3, 1> p_wv = const_cast<const EKFState_T&>(*latestState).template Get<StateDefinition_T::p_wv>();

        p_vc_p = position_handler_->GetPositionMeasurement(); //This is potentially different for both sensors but has same name

        //position is given by last state
        //we do not touch pose sensor so we need to recompute transformation of position sensor

        //rotational part doesnt really exist since gps has no orientation
        //compute new tranform for position (translational part)
        p_ip = q.toRotationMatrix().inverse()*(p_vc_p-p);

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
    else
    {
        MSF_WARN_STREAM("Unknown sensor ID:"<<sensorID);
    }     
    return;
  }



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
