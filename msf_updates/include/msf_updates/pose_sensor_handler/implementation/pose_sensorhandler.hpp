/*
 * Copyright (C) 2012-2013 Simon Lynen, ASL, ETH Zurich, Switzerland
 * You can contact the author at <slynen at ethz dot ch>
 * Copyright (C) 2011-2012 Stephan Weiss, ASL, ETH Zurich, Switzerland
 * You can contact the author at <stephan dot weiss at ieee dot org>
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
#include <msf_core/eigen_utils.h>
#include <msf_core/msf_types.h>
#include <std_srvs/Empty.h>
#include <sensor_fusion_comm/AddListener.h>
#include <sensor_fusion_comm/EvalListener.h>

#ifndef POSE_SENSORHANDLER_HPP_
#define POSE_SENSORHANDLER_HPP_

namespace msf_pose_sensor {
typedef msf_updates::EKFState EKFState_T;
typedef EKFState_T::StateSequence_T StateSequence_T;
typedef EKFState_T::StateDefinition_T StateDefinition_T;
template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
PoseSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::PoseSensorHandler(
    MANAGER_TYPE& mng, std::string topic_namespace,
    std::string parameternamespace, bool distortmeas)
    : SensorHandler<msf_updates::EKFState>(mng, topic_namespace,
                                           parameternamespace),
      n_zp_(1e-6),
      n_zq_(1e-6),
      delay_(0),
      timestamp_previous_pose_(0),
      curr_reset_savetime_(0),
      needs_reinit_(false) {
  ros::NodeHandle pnh("~/pose_sensor");

  MSF_INFO_STREAM(
      "Loading parameters for pose sensor from namespace: "
          << pnh.getNamespace());

  pnh.param("pose_absolute_measurements", provides_absolute_measurements_,
            true);
  pnh.param("pose_measurement_world_sensor", measurement_world_sensor_, true);
  pnh.param("pose_use_fixed_covariance", use_fixed_covariance_, false);
  pnh.param("pose_measurement_minimum_dt", pose_measurement_minimum_dt_, 0.05);
  //params for outlierrejection
  pnh.param("enable_mah_outlier_rejection", enable_mah_outlier_rejection_, false);
  pnh.param("mah_threshold", mah_threshold_, msf_core::kDefaultMahThreshold_);
  //mah_threshold must not be <1 for two reasons:
  //numerical stability
  //makes no sense to expecte most measurement to have mahalanobis distance < 1
  if(mah_threshold_<1.0)
  {
      MSF_WARN_STREAM("mah_threshold set to be < 1. Correcting to 1");
      mah_threshold_=1.0;
  }
  //params for noise estimation
  pnh.param("enable_noise_estimation", enable_noise_estimation_, false);
  pnh.param("noise_estimation_discount_factor", average_discount_factor_, 1.0);
  pnh.param("max_noise_threshold", max_noise_threshold_, 10.0);
  running_maha_dist_average_=msf_core::desiredNoiseLevel_*mah_threshold_;

  //this is for noise estimation using a neural network
  pnh.param("use_nn_noise_estimation", use_nn_noise_estimation_, false);
  std::string emptystr="";
  pnh.param("tf_key", tf_key_, emptystr);
  pnh.param("tf_network_path", tf_network_path_, emptystr);
  pnh.param("tf_network_weights", tf_network_weights_, emptystr);
  pnh.param("tf_max_memory", tf_max_memory_, 0);
  pnh.param("tf_eval_frequency", tf_eval_frequency_, 1);
  if(use_nn_noise_estimation_)
  {
    enable_noise_estimation_=true;
    MSF_INFO_STREAM("using NN to predict noise parameters with update frequency "<<tf_eval_frequency_);
  }
  else
  {
      tf_key_="";
  }

  //params for divergence recovery
  pnh.param("enable_divergence_recovery", enable_divergence_recovery_, true);
  pnh.param("divergence_rejection_limit", rejection_divergence_threshold_, msf_core::defaultRejectionDivergenceThreshold_);
  pnh.param("use_reset_to_pose", use_reset_to_pose_, true);
  pnh.param("use_transform_recovery", use_transform_recovery_, false);
  pnh.param("transform_recovery_noise_p", transform_recovery_noise_p_,0.0);
  pnh.param("transform_recovery_noise_q", transform_recovery_noise_q_,0.0);
  pnh.param("transform_anealing_steps", transform_anealing_steps_, 1);
  transform_curr_anealing_steps_ = 0;
  //this is how to get config
  MANAGER_TYPE* mngr = dynamic_cast<MANAGER_TYPE*>(&manager_);
  transform_base_noise_p_ = mngr->config_.pose_noise_p_wv;
  transform_base_noise_q_ = mngr->config_.pose_noise_q_wv;



  MSF_INFO_STREAM_COND(measurement_world_sensor_, "Pose sensor is interpreting "
                       "measurement as sensor w.r.t. world");
  MSF_INFO_STREAM_COND(
      !measurement_world_sensor_,
      "Pose sensor is interpreting measurement as world w.r.t. "
      "sensor (e.g. ethzasl_ptam)");

  MSF_INFO_STREAM_COND(use_fixed_covariance_, "Pose sensor is using fixed "
                       "covariance");
  MSF_INFO_STREAM_COND(!use_fixed_covariance_,
                       "Pose sensor is using covariance "
                       "from sensor");

  MSF_INFO_STREAM_COND(provides_absolute_measurements_,
                       "Pose sensor is handling "
                       "measurements as absolute values");
  MSF_INFO_STREAM_COND(!provides_absolute_measurements_, "Pose sensor is "
                       "handling measurements as relative values");

  if(enable_mah_outlier_rejection_)
  {
	  MSF_INFO_STREAM("Pose sensor is using outlier rejection with threshold: " <<
	  mah_threshold_);
  }
  if(enable_noise_estimation_)
  {
      MSF_INFO_STREAM("Pose sensor is using noise estimation with discout factor:"<<
      average_discount_factor_);
  }
  if(enable_divergence_recovery_)
  {
      MSF_INFO_STREAM("Pose sensor is using divergence recovery with rejection limit:"<<
      rejection_divergence_threshold_);
  }
  if(use_transform_recovery_)
  {
      MSF_INFO_STREAM("Pose sensor uses transform recovery with noise p:"<<transform_recovery_noise_p_<<
    " and q:"<<transform_recovery_noise_q_);
  }  
  if(use_reset_to_pose_)
  {
      MSF_INFO_STREAM("Pose sensor is reseting rovio to pose");
  }

  //This part is for creating training sets for noise estimation LSTM
  bool create_ts;
  pnh.param("create_training_set", create_ts, false);
  if(create_ts)
  {
      std::string path_to_ts;
      std::string default_ts="default.txt"; //for some reason I cant put ""
      pnh.param("path_to_training_set", path_to_ts, default_ts);
      ts_IO_outfile_obj_.open(path_to_ts, std::ios_base::app);
      //for postprocessing add this line at beginning of every training sequence
      ts_IO_outfile_obj_<<"starting new training sequence"<<std::endl;
      ts_IO_outfile_=&ts_IO_outfile_obj_;
  }
  
  ros::NodeHandle nh("msf_updates/" + topic_namespace);
  subPoseWithCovarianceStamped_ =
      nh.subscribe < geometry_msgs::PoseWithCovarianceStamped
          > ("pose_with_covariance_input", 20, &PoseSensorHandler::MeasurementCallback, this);
  subTransformStamped_ = nh.subscribe < geometry_msgs::TransformStamped
      > ("transform_input", 20, &PoseSensorHandler::MeasurementCallback, this);
  subPoseStamped_ = nh.subscribe < geometry_msgs::PoseStamped
      > ("pose_input", 20, &PoseSensorHandler::MeasurementCallback, this);

  z_p_.setZero();
  z_q_.setIdentity();

  if (distortmeas) {
    Eigen::Vector3d meanpos;
    double distortpos_mean;
    pnh.param("distortpos_mean", distortpos_mean, 0.0);
    meanpos.setConstant(distortpos_mean);

    Eigen::Vector3d stddevpos;
    double distortpos_stddev;
    pnh.param("distortpos_stddev", distortpos_stddev, 0.0);
    stddevpos.setConstant(distortpos_stddev);

    Eigen::Vector3d meanatt;
    double distortatt_mean;
    pnh.param("distortatt_mean", distortatt_mean, 0.0);
    meanatt.setConstant(distortatt_mean);

    Eigen::Vector3d stddevatt;
    double distortatt_stddev;
    pnh.param("distortatt_stddev", distortatt_stddev, 0.0);
    stddevatt.setConstant(distortatt_stddev);

    double distortscale_mean;
    pnh.param("distortscale_mean", distortscale_mean, 0.0);
    double distortscale_stddev;
    pnh.param("distortscale_stddev", distortscale_stddev, 0.0);

    distorter_.reset(
        new msf_updates::PoseDistorter(meanpos, stddevpos, meanatt, stddevatt,
                                       distortscale_mean, distortscale_stddev));
  }
}

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void PoseSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::SetNoises(double n_zp,
                                                                  double n_zq) {
  n_zp_ = n_zp;
  n_zq_ = n_zq;
}

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void PoseSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::SetDelay(double delay) {
  delay_ = delay;
}

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void PoseSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::ProcessPoseMeasurement(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr & msg) {
  received_first_measurement_ = true;
  if(needs_reinit_)
  {
      if(curr_reset_savetime_>0)
      {
          curr_reset_savetime_--;
          return;
      }
      manager_.Initsingle(this->sensorID);
      needs_reinit_=false;
      return;
  }
  // Get the fixed states.
  int fixedstates = 0;
  static_assert(msf_updates::EKFState::nStateVarsAtCompileTime < 32, "Your state "
      "has more than 32 variables. The code needs to be changed here to have a "
      "larger variable to mark the fixed_states");
  // Do not exceed the 32 bits of int.

  if(collect_for_init_)
  {
      Eigen::Vector4d newpoint;
      //set values of newpoint
      newpoint(0)=msg->pose.pose.position.x;
      newpoint(1)=msg->pose.pose.position.y;
      newpoint(2)=msg->pose.pose.position.z;
      newpoint(3)=msg->header.stamp.toSec() - delay_;
      init_points_.push_back(newpoint);
      if(init_points_.size()>=2)
      {
        total_init_movement_+=(init_points_[init_points_.size()-2].head(3)-init_points_[init_points_.size()-1].head(3)).norm();
      }
      //if not ready yet need to check wether ready with new meas
      //MSF_INFO_STREAM("pose calling");
      if(!ready_for_init_)
      {
          if(InitPointsReady())
          {
            //set ready true and call manager initstable
            ready_for_init_=true;
            MANAGER_TYPE* mngr = dynamic_cast<MANAGER_TYPE*>(&manager_);
            mngr->InitStable();
          }
      }
      //the reason we need this is because it might think it had enough distance because of an outlier
      //and like this both sensors need an outlier on the same frame
      else
      {
          if(!InitPointsReady())
          {
              ready_for_init_=false;
          }
      }  
        
      //return;
  }
  // Get all the fixed states and set flag bits.
  MANAGER_TYPE* mngr = dynamic_cast<MANAGER_TYPE*>(&manager_);

  // TODO(acmarkus): if we have multiple sensor handlers, they all share the same dynparams,
  // which me maybe don't want. E.g. if we have this for multiple AR Markers, we
  // may want to keep one fix --> move this to fixed parameters? Could be handled
  // with parameter namespace then.
  if (mngr) {
    if (mngr->Getcfg().pose_fixed_scale) {
      fixedstates |= 1 << MEASUREMENT_TYPE::AuxState::L;
    }
    if (mngr->Getcfg().pose_fixed_p_ic) {
      fixedstates |= 1 << MEASUREMENT_TYPE::AuxState::p_ic;
    }
    if (mngr->Getcfg().pose_fixed_q_ic) {
      fixedstates |= 1 << MEASUREMENT_TYPE::AuxState::q_ic;
    }
    if (mngr->Getcfg().pose_fixed_p_wv) {
      fixedstates |= 1 << MEASUREMENT_TYPE::AuxState::p_wv;
    }
    if (mngr->Getcfg().pose_fixed_q_wv) {
      fixedstates |= 1 << MEASUREMENT_TYPE::AuxState::q_wv;
    }
  }


  shared_ptr<MEASUREMENT_TYPE> meas(new MEASUREMENT_TYPE(
      n_zp_, n_zq_, measurement_world_sensor_, use_fixed_covariance_,
      provides_absolute_measurements_, this->sensorID,
      enable_mah_outlier_rejection_, mah_threshold_, &running_maha_dist_average_,
      average_discount_factor_, &n_rejected_, &n_curr_rejected_,
      &n_accepted_, tf_key_, ts_IO_outfile_, fixedstates, distorter_));

  meas->MakeFromSensorReading(msg, msg->header.stamp.toSec() - delay_);

  z_p_ = meas->z_p_;  //store this for the init procedure
  z_q_ = meas->z_q_;
 
  this->manager_.msf_core_->AddMeasurement(meas);
    
  //this function should check wether too many measurements have been rejected -> increase noise meas
  //or wether this sensor is currently diverging -> use recovery and increase noise meas
  if(enable_noise_estimation_&&!collect_for_init_)
  {
      //+1 is for not evaluating with 0 messages
      if(use_nn_noise_estimation_)
      {
          n_noise_estimation_point_++;
          //MSF_INFO_STREAM((int)(n_accepted_+n_rejected_+1)%tf_eval_frequency_<<"val "<<n_accepted_);
          if(n_noise_estimation_point_>tf_eval_frequency_)
            {
            //make service call to evaluation
            MSF_WARN_STREAM("evaluating NN"<<n_noise_estimation_point_<<" "<<tf_eval_frequency_);
            n_noise_estimation_point_-=tf_eval_frequency_;
            ros::NodeHandle ntemp;
            ros::ServiceClient clienttemp = ntemp.serviceClient<sensor_fusion_comm::EvalListener>("eval_node/eval_listener");
            sensor_fusion_comm::EvalListener srvtemp;
            srvtemp.request.key = tf_key_;
            
            if(clienttemp.call(srvtemp)&&srvtemp.response.output[0]!=-1)
            {
                MSF_INFO_STREAM("new noise"<<srvtemp.response.output[0]<<"and "<<srvtemp.response.output[1]);
                mngr->config_.pose_noise_meas_p = std::max(0.05, std::min(srvtemp.response.output[0], this->GetMaxNoiseThreshold()));
                mngr->config_.pose_noise_meas_q = std::max(0.02, std::min(srvtemp.response.output[1], this->GetMaxNoiseThreshold()/2));
                //set noise
                this->SetNoises(mngr->config_.pose_noise_meas_p, mngr->config_.pose_noise_meas_q);
                return;
            }
          }
      }
      else
      {
        //if running average is larger than upperNoiseLimit of threshold recompute noise based on this
        if(running_maha_dist_average_>=msf_core::upperNoiseLimit_*mah_threshold_ || running_maha_dist_average_<=msf_core::lowerNoiseLimit_*mah_threshold_)
        {

                double tempfactor=(1.0+2.0*(running_maha_dist_average_/mah_threshold_-msf_core::desiredNoiseLevel_));
                //use a factor based on val
                //if factor larger one we want:
                //not surpass max threshold
                //increase to fixed amount if it was too small (~0) before
                if (tempfactor>1.0)
                {
                    mngr->config_.pose_noise_meas_p = std::max(0.05, std::min(mngr->config_.pose_noise_meas_p*tempfactor, this->GetMaxNoiseThreshold()));
                    //q noise should be smaller
                    mngr->config_.pose_noise_meas_q = std::max(0.02, std::min(mngr->config_.pose_noise_meas_q*tempfactor, this->GetMaxNoiseThreshold()/2));
                }
                //no additional constraints
                else
                {
                    mngr->config_.pose_noise_meas_p = mngr->config_.pose_noise_meas_p*tempfactor, this->GetMaxNoiseThreshold();
                    mngr->config_.pose_noise_meas_q = mngr->config_.pose_noise_meas_q*tempfactor, this->GetMaxNoiseThreshold();
                }
                MSF_INFO_STREAM("Changing Noise measurement p to:"<<mngr->config_.pose_noise_meas_p);
                this->SetNoises(mngr->config_.pose_noise_meas_p, mngr->config_.pose_noise_meas_q);
                //probably reset makes sense here since we basically start again
                n_accepted_=0.0;
                n_rejected_=0.0;
                running_maha_dist_average_=msf_core::desiredNoiseLevel_*mah_threshold_;
                return;
        }
      }

  }

  //here rovio is (most likely) diverging currently. Reset rovio and increase noise meas to compensate
  if(enable_divergence_recovery_ && n_curr_rejected_>rejection_divergence_threshold_)
  {
      MSF_WARN_STREAM("too many measurements have been rejected back to back -> increasing stability parameters and reseting");
      
      n_accepted_ = 0.0;
      n_rejected_ = 0.0;
      n_curr_rejected_ = 0.0;
      
      if(enable_noise_estimation_)
      {
        double tempfactor=(1.0+2.0*(running_maha_dist_average_/mah_threshold_-msf_core::desiredNoiseLevel_));
        //use a factor based on val
        //if factor larger one we want:
        //not surpass max threshold
        //increase to fixed amount if it was too small (~0) before
        if (tempfactor>1.0)
        {
            mngr->config_.pose_noise_meas_p = std::max(0.05, std::min(mngr->config_.pose_noise_meas_p*tempfactor, this->GetMaxNoiseThreshold()));
            //q noise should be smaller
            mngr->config_.pose_noise_meas_q = std::max(0.02, std::min(mngr->config_.pose_noise_meas_q*tempfactor, this->GetMaxNoiseThreshold()/2));
        }
        //no additional constraints
        else
        {
            mngr->config_.pose_noise_meas_p = mngr->config_.pose_noise_meas_p*tempfactor, this->GetMaxNoiseThreshold();
            mngr->config_.pose_noise_meas_q = mngr->config_.pose_noise_meas_q*tempfactor, this->GetMaxNoiseThreshold();
        }
        
        MSF_INFO_STREAM("Changing Noise measurement p to:"<<mngr->config_.pose_noise_meas_p);
        this->SetNoises(mngr->config_.pose_noise_meas_p, mngr->config_.pose_noise_meas_q);
        running_maha_dist_average_=msf_core::desiredNoiseLevel_*mah_threshold_;
      }
      //this completely resets rovio
      if(!use_reset_to_pose_)
      {
        ros::NodeHandle ntemp;
        ros::ServiceClient clienttemp = ntemp.serviceClient<std_srvs::Empty>("rovio/reset");
        std_srvs::Empty srvtemp;
        clienttemp.call(srvtemp);
        needs_reinit_=true; //setting this to true will cause it to reinit on next measurement
        curr_reset_savetime_=msf_core::rovioResetSaveTime; //this will delay init by n measurements to account for some measurements bein buffered
        received_first_measurement_=false;
      }
      //try to reset rovio using rovios built in function "reset_to_pose"
      //this should be better than complete reset unless msf estimate is bad for some reason
      else
      {
        ros::NodeHandle ntemp;
        ros::ServiceClient clienttemp = ntemp.serviceClient<rovio::SrvResetToPose>("rovio/reset_to_pose");
        rovio::SrvResetToPose srvtemp;
        //this should be: shared_ptr<EKFState_T>&
        //but no access to EKFState_T
        const shared_ptr<EKFState_T>& latestState = this->manager_.msf_core_->GetLastState();
        //this is the last state
        const Eigen::Quaternion<double> q = const_cast<const EKFState_T&>(*latestState).template Get<StateDefinition_T::q>();
        const Eigen::Matrix<double, 3, 1> p = const_cast<const EKFState_T&>(*latestState).template Get<StateDefinition_T::p>();
        //we also need the transforms because we need to transform the current pose to rovio's frame (i.e. the inverse transform from the initialization)
        const Eigen::Matrix<double, 3, 1> p_ic = const_cast<const EKFState_T&>(*latestState).template Get<StateDefinition_T::p_ic>();
        const Eigen::Quaternion<double> q_ic = const_cast<const EKFState_T&>(*latestState).template Get<StateDefinition_T::q_ic>();
        const Eigen::Matrix<double, 3, 1> p_wv = const_cast<const EKFState_T&>(*latestState).template Get<StateDefinition_T::p_wv>();
        const Eigen::Quaternion<double> q_wv = const_cast<const EKFState_T&>(*latestState).template Get<StateDefinition_T::q_wv>();
        const double scale = (const_cast<const EKFState_T&>(*latestState).template Get<StateDefinition_T::p_wv>())(0,0);
        //this is the pose in rovio frame
        const Eigen::Quaternion<double> q_rovio ((q_ic.conjugate()*q.conjugate()*q_wv.conjugate()).conjugate().normalized());
        const Eigen::Matrix<double, 3, 1> p_rovio = q_wv*(p - p_wv + q * p_ic);


        srvtemp.request.T_WM.position.x = p_rovio(0,0);
        srvtemp.request.T_WM.position.y = p_rovio(1,0);
        srvtemp.request.T_WM.position.z = p_rovio(2,0);
        if(q_rovio.w()>0)
        {
            srvtemp.request.T_WM.orientation.w = q_rovio.w();
            srvtemp.request.T_WM.orientation.x = q_rovio.x();
            srvtemp.request.T_WM.orientation.y = q_rovio.y();
            srvtemp.request.T_WM.orientation.z = q_rovio.z();
        }
        else
        {
            srvtemp.request.T_WM.orientation.w = -q_rovio.w();
            srvtemp.request.T_WM.orientation.x = -q_rovio.x();
            srvtemp.request.T_WM.orientation.y = -q_rovio.y();
            srvtemp.request.T_WM.orientation.z = -q_rovio.z();
        }
        clienttemp.call(srvtemp);
      }
      
      //if set true this increases transform noise and uses simulated anneahling to lower it to initial value
      if(use_transform_recovery_)
      {

          mngr->config_.pose_noise_p_wv = std::min(mngr->config_.pose_noise_meas_p/2.0, transform_recovery_noise_p_);
          mngr->config_.pose_noise_q_wv = std::min(mngr->config_.pose_noise_meas_q/2.0,transform_recovery_noise_q_);
          transform_curr_anealing_steps_ = transform_anealing_steps_-2;
      }
      MSF_WARN_STREAM("reinitializing rovio");
      return;
    }
    if(use_transform_recovery_ && transform_curr_anealing_steps_>0)
    {
       
        mngr->config_.pose_noise_p_wv -= (transform_recovery_noise_p_-transform_base_noise_p_)/transform_anealing_steps_;
        mngr->config_.pose_noise_q_wv -= (transform_recovery_noise_q_-transform_base_noise_q_)/transform_anealing_steps_;
        transform_curr_anealing_steps_-=1;
    }  
}
template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void PoseSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::MeasurementCallback(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr & msg) {

  this->SequenceWatchDog(msg->header.seq,
                         subPoseWithCovarianceStamped_.getTopic());
  MSF_INFO_STREAM_ONCE(
      "*** pose sensor got first measurement from topic "
          << this->topic_namespace_ << "/"
          << subPoseWithCovarianceStamped_.getTopic() << " ***");
  ProcessPoseMeasurement(msg);
}

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void PoseSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::MeasurementCallback(
    const geometry_msgs::TransformStampedConstPtr & msg) {
  this->SequenceWatchDog(msg->header.seq, subTransformStamped_.getTopic());
  MSF_INFO_STREAM_ONCE(
      "*** pose sensor got first measurement from topic "
          << this->topic_namespace_ << "/" << subTransformStamped_.getTopic()
          << " ***");

  double time_now = msg->header.stamp.toSec();
  const double epsilon = 0.001; // Small time correction to avoid rounding errors in the timestamps.
  if (time_now - timestamp_previous_pose_ <= pose_measurement_minimum_dt_ - epsilon) {
    MSF_WARN_STREAM_THROTTLE(30, "Pose measurement throttling is on, dropping messages"
                             "to be below " +
                             std::to_string(1/pose_measurement_minimum_dt_) + " Hz");
    return;
  }

  timestamp_previous_pose_ = time_now;

  geometry_msgs::PoseWithCovarianceStampedPtr pose(
      new geometry_msgs::PoseWithCovarianceStamped());

  if (!use_fixed_covariance_)  // Take covariance from sensor.
  {
    MSF_WARN_STREAM_THROTTLE(
        2,
        "Provided message type without covariance but set fixed_covariance == "
        "false at the same time. Discarding message.");
    return;
  }

  // Fixed covariance will be set in measurement class -> MakeFromSensorReadingImpl.
  pose->header = msg->header;

  pose->pose.pose.position.x = msg->transform.translation.x;
  pose->pose.pose.position.y = msg->transform.translation.y;
  pose->pose.pose.position.z = msg->transform.translation.z;

  pose->pose.pose.orientation.w = msg->transform.rotation.w;
  pose->pose.pose.orientation.x = msg->transform.rotation.x;
  pose->pose.pose.orientation.y = msg->transform.rotation.y;
  pose->pose.pose.orientation.z = msg->transform.rotation.z;

  ProcessPoseMeasurement(pose);
}

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void PoseSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::MeasurementCallback(
    const geometry_msgs::PoseStampedConstPtr & msg) {
  this->SequenceWatchDog(msg->header.seq, subPoseStamped_.getTopic());
  MSF_INFO_STREAM_ONCE(
      "*** pose sensor got first measurement from topic "
          << this->topic_namespace_ << "/" << subPoseStamped_.getTopic()
          << " ***");

  geometry_msgs::PoseWithCovarianceStampedPtr pose(
      new geometry_msgs::PoseWithCovarianceStamped());

  if (!use_fixed_covariance_)  // Take covariance from sensor.
  {
    MSF_WARN_STREAM_THROTTLE(
        2,
        "Provided message type without covariance but set fixed_covariance =="
        "false at the same time. Discarding message.");
    return;
  }

  // Fixed covariance will be set in measurement class -> MakeFromSensorReadingImpl.

  pose->header = msg->header;

  pose->pose.pose = msg->pose;

  ProcessPoseMeasurement(pose);
}
}  // namespace msf_pose_sensor
#endif  // POSE_SENSORHANDLER_HPP_
