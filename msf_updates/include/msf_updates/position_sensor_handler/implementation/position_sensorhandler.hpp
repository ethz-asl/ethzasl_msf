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
#ifndef POSITION_SENSORHANDLER_HPP_
#define POSITION_SENSORHANDLER_HPP_
#include <msf_core/msf_types.h>
#include <msf_core/eigen_utils.h>
#include <msf_core/gps_conversion.h>
#include <sensor_fusion_comm/AddListener.h>
#include <sensor_fusion_comm/EvalListener.h>

namespace msf_position_sensor {
template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
PositionSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::PositionSensorHandler(
    MANAGER_TYPE& meas, std::string topic_namespace,
    std::string parameternamespace)
    : SensorHandler<msf_updates::EKFState>(meas, topic_namespace,
                                           parameternamespace),
      n_zp_(1e-6),
      delay_(0) {
  MSF_WARN_STREAM("initialized position sensor handler");
  ros::NodeHandle pnh("~/position_sensor");
  pnh.param("position_use_fixed_covariance", use_fixed_covariance_, false);
  pnh.param("position_absolute_measurements", provides_absolute_measurements_,
            false);
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
  //MSF_WARN_STREAM(running_maha_dist_average_);
  //params for divergence recovery
  pnh.param("enable_divergence_recovery", enable_divergence_recovery_, false);
  pnh.param("divergence_rejection_limit", rejection_divergence_threshold_, msf_core::defaultRejectionDivergenceThreshold_);

  MSF_INFO_STREAM_COND(use_fixed_covariance_, "Position sensor is using fixed "
                       "covariance");
  MSF_INFO_STREAM_COND(!use_fixed_covariance_, "Position sensor is using "
                       "covariance from sensor");

  MSF_INFO_STREAM_COND(provides_absolute_measurements_, "Position sensor is "
                       "handling measurements as absolute values");
  MSF_INFO_STREAM_COND(!provides_absolute_measurements_, "Position sensor is "
                       "handling measurements as relative values");

  if(enable_mah_outlier_rejection_)
  {
	  MSF_INFO_STREAM("Position sensor is using outlier rejection with threshold: " <<
	  mah_threshold_);
  }
  if(enable_noise_estimation_)
  {
      MSF_INFO_STREAM("Position sensor is using noise estimation with discout factor:"<<
      average_discount_factor_);
  }
  if(enable_divergence_recovery_)
  {
      MSF_INFO_STREAM("Position sensor is using divergence recovery with rejection limit:"<<
      rejection_divergence_threshold_);
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
  ros::NodeHandle nh("msf_updates");

  subPointStamped_ =
      nh.subscribe<geometry_msgs::PointStamped>
  ("position_input", 20, &PositionSensorHandler::MeasurementCallback, this);
  subTransformStamped_ =
      nh.subscribe<geometry_msgs::TransformStamped>
  ("transform_input", 20, &PositionSensorHandler::MeasurementCallback, this);
  subNavSatFix_ =
      nh.subscribe<sensor_msgs::NavSatFix>
  ("navsatfix_input", 20, &PositionSensorHandler::MeasurementCallback, this);

  z_p_.setZero();

}

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void PositionSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::SetNoises(
    double n_zp) {
  n_zp_ = n_zp;
}

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void PositionSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::SetDelay(
    double delay) {
  delay_ = delay;
}

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void PositionSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::AdjustGPSZReference(
    double current_z) {
  gpsConversion_.AdjustReference(z_p_(2) - current_z);
}

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void PositionSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::ProcessPositionMeasurement(
    const sensor_fusion_comm::PointWithCovarianceStampedConstPtr& msg) {
  received_first_measurement_ = true;

  // Get the fixed states.
  int fixedstates = 0;
  static_assert(msf_updates::EKFState::nStateVarsAtCompileTime < 32, "Your state "
      "has more than 32 variables. The code needs to be changed here to have a "
      "larger variable to mark the fixed_states");
  // Do not exceed the 32 bits of int.

  if (!use_fixed_covariance_ && msg->covariance[0] == 0)  // Take covariance from sensor.
      {
    MSF_WARN_STREAM_THROTTLE(
        2, "Provided message type without covariance but set "
        "fixed_covariance=false at the same time. Discarding message.");
    return;
  }

  if(collect_for_init_)
  {
      Eigen::Vector4d newpoint;
      //set values of newpoint
      newpoint(0)=msg->point.x;
      newpoint(1)=msg->point.y;
      newpoint(2)=msg->point.z;
      newpoint(3)=msg->header.stamp.toSec() - delay_;
      //MSF_INFO_STREAM(newpoint);
      init_points_.push_back(newpoint);
      if(init_points_.size()>=2)
      {
        total_init_movement_+=(init_points_[init_points_.size()-2].head(3)-init_points_[init_points_.size()-1].head(3)).norm();
      }
      //if not ready yet need to check wether ready with new meas
      //MSF_INFO_STREAM("position calling");
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

  if (mngr) {
    if (mngr->Getcfg().position_fixed_p_ip) {
      fixedstates |= 1 << msf_updates::EKFState::StateDefinition_T::p_ip;
    }
  }
  

  //MSF_WARN_STREAM("maha dist average before meas:"<<running_maha_dist_average_);
  shared_ptr<MEASUREMENT_TYPE> meas(new MEASUREMENT_TYPE(
      n_zp_, use_fixed_covariance_, provides_absolute_measurements_,
      this->sensorID, fixedstates, enable_mah_outlier_rejection_,
      mah_threshold_, &running_maha_dist_average_, average_discount_factor_,
      &n_rejected_, &n_curr_rejected_, &n_accepted_, tf_key_, ts_IO_outfile_));

  meas->MakeFromSensorReading(msg, msg->header.stamp.toSec() - delay_);
  

  z_p_ = meas->z_p_;  // Store this for the init procedure.
  
  this->manager_.msf_core_->AddMeasurement(meas);

  
  if(enable_noise_estimation_&&!collect_for_init_)
  {
      //+1 is for not evaluating with 0 messages
      if(use_nn_noise_estimation_)
      {
          n_noise_estimation_point_++;
          //MSF_INFO_STREAM((int)(n_accepted_+n_rejected_+1)%tf_eval_frequency_<<"val "<<n_accepted_);
          if(n_noise_estimation_point_>=tf_eval_frequency_)
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
                MSF_INFO_STREAM("new noise"<<srvtemp.response.output[0]);
                mngr->config_.position_noise_meas = std::max(0.05, std::min(srvtemp.response.output[0], this->GetMaxNoiseThreshold()));
                //set noise
                this->SetNoises(mngr->config_.position_noise_meas);
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
                    mngr->config_.position_noise_meas = std::max(0.05, std::min(mngr->config_.position_noise_meas*tempfactor, this->GetMaxNoiseThreshold()));
            }
            //if we decrease we just use factor (without additional constraints)  
            else
            {
                mngr->config_.position_noise_meas = mngr->config_.position_noise_meas*tempfactor;
            }
            
            MSF_INFO_STREAM("Changing position Noise measurement to:"<<mngr->config_.position_noise_meas);
            this->SetNoises(mngr->config_.position_noise_meas);
            //MSF_WARN_STREAM("too big:"<<running_maha_dist_average_<<"..."<<msf_core::upperNoiseLimit_*mah_threshold_);
            //manager_.IncreaseNoise(this->sensorID, running_maha_dist_average_/mah_threshold_);
            //probably reset makes sense here since we basically start again
            n_accepted_=0.0;
            n_rejected_=0.0;
            running_maha_dist_average_=msf_core::desiredNoiseLevel_*mah_threshold_;
            //manager_.Initsingle(this->sensorID);
            return;
        }
      }
  }
  
  //this usually makes no sense (i.e. set recovery=false) since sensor will mostly recover on its own 
  if(enable_divergence_recovery_ && n_curr_rejected_>rejection_divergence_threshold_)
  {
      double tempfactor=(1.0+2.0*(running_maha_dist_average_/mah_threshold_-msf_core::desiredNoiseLevel_));
      MSF_WARN_STREAM("too many measurements have been rejected back to back -> increasing stability parameters and reseting");
      n_accepted_ = 0.0;
      n_rejected_ = 0.0;
      n_curr_rejected_ = 0.0;
      if(enable_noise_estimation_)
      {
        //use a factor based on val
        //if factor larger one we want:
        //not surpass max threshold
        //increase to fixed amount if it was too small (~0) before
        if (tempfactor>1.0)
        {
            mngr->config_.position_noise_meas = std::max(0.05, std::min(mngr->config_.position_noise_meas*tempfactor, this->GetMaxNoiseThreshold()));
        }
        //if we decrease we just use factor (without additional constraints)  
        else
        {
            mngr->config_.position_noise_meas = mngr->config_.position_noise_meas*tempfactor;
        }
        
        MSF_INFO_STREAM("Changing position Noise measurement to:"<<mngr->config_.position_noise_meas);
        this->SetNoises(mngr->config_.position_noise_meas);
      }
      running_maha_dist_average_=msf_core::desiredNoiseLevel_*mah_threshold_;
      manager_.Initsingle(this->sensorID);
      return;
  }  
  


}

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void PositionSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::MeasurementCallback(
    const geometry_msgs::PointStampedConstPtr & msg) {
  this->SequenceWatchDog(msg->header.seq, subPointStamped_.getTopic());

  MSF_INFO_STREAM_ONCE(
      "*** position sensor got first measurement from topic "
          << this->topic_namespace_ << "/" << subPointStamped_.getTopic()
          << " ***");

  sensor_fusion_comm::PointWithCovarianceStampedPtr pointwCov(
      new sensor_fusion_comm::PointWithCovarianceStamped);
  pointwCov->header = msg->header;
  pointwCov->point = msg->point;

  ProcessPositionMeasurement(pointwCov);
}

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void PositionSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::MeasurementCallback(
    const geometry_msgs::TransformStampedConstPtr & msg) {
  this->SequenceWatchDog(msg->header.seq, subTransformStamped_.getTopic());

  MSF_INFO_STREAM_ONCE(
      "*** position sensor got first measurement from topic "
          << this->topic_namespace_ << "/" << subTransformStamped_.getTopic()
          << " ***");

  if (msg->header.seq % 5 != 0) {  //slow down vicon
    MSF_WARN_STREAM_ONCE("Measurement throttling is on, dropping every but the "
                         "5th message");
    return;
  }

  sensor_fusion_comm::PointWithCovarianceStampedPtr pointwCov(
      new sensor_fusion_comm::PointWithCovarianceStamped);
  pointwCov->header = msg->header;

  // Fixed covariance will be set in measurement class -> MakeFromSensorReadingImpl.
  pointwCov->point.x = msg->transform.translation.x;
  pointwCov->point.y = msg->transform.translation.y;
  pointwCov->point.z = msg->transform.translation.z;

  ProcessPositionMeasurement(pointwCov);
}

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void PositionSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::MeasurementCallback(
    const sensor_msgs::NavSatFixConstPtr& msg) {
  this->SequenceWatchDog(msg->header.seq, subNavSatFix_.getTopic());

  MSF_INFO_STREAM_ONCE(
      "*** position sensor got first measurement from topic "
          << this->topic_namespace_ << "/" << subNavSatFix_.getTopic()
          << " ***");

  // Fixed covariance will be set in measurement class -> MakeFromSensorReadingImpl.
  static bool referenceinit = false;  //TODO (slynen): Dynreconf reset reference.
  if (!referenceinit) {
    gpsConversion_.InitReference(msg->latitude, msg->longitude, msg->altitude);
    MSF_WARN_STREAM(
        "Initialized GPS reference of topic: " << this->topic_namespace_ << "/"
            << subNavSatFix_.getTopic() << " to lat/lon/alt: [" << msg->latitude
            << ", " << msg->longitude << ", " << msg->altitude << "]");
    referenceinit = true;
  }

  msf_core::Vector3 ecef = gpsConversion_.WGS84ToECEF(msg->latitude,
                                                      msg->longitude,
                                                      msg->altitude);
  msf_core::Vector3 enu = gpsConversion_.ECEFToENU(ecef);

  sensor_fusion_comm::PointWithCovarianceStampedPtr pointwCov(
      new sensor_fusion_comm::PointWithCovarianceStamped);
  pointwCov->header = msg->header;

  // Store the ENU data in the position fields.
  pointwCov->point.x = enu[0];
  pointwCov->point.y = enu[1];
  pointwCov->point.z = enu[2];

  // Get the covariance TODO (slynen): handle the cases differently!
  if (msg->position_covariance_type
      == sensor_msgs::NavSatFix::COVARIANCE_TYPE_KNOWN) {
    pointwCov->covariance = msg->position_covariance;
  } else if (msg->position_covariance_type
      == sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN) {
    pointwCov->covariance = msg->position_covariance;
  } else if (msg->position_covariance_type
      == sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED) {  // From DOP.
    pointwCov->covariance = msg->position_covariance;
  }

  ProcessPositionMeasurement(pointwCov);
}
}  // namespace msf_position_sensor
#endif  // POSITION_SENSORHANDLER_HPP_
