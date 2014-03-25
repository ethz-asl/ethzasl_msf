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

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <msf_distort/MSF_DistortConfig.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/server.h>
#include <msf_core/msf_macros.h>

struct CallbackHandler {

  typedef msf_distort::MSF_DistortConfig Config_T;
  Config_T config_;
  ros::Publisher pubPoseWithCovarianceStamped_;
  ros::Publisher pubTransformStamped_;
  ros::Publisher pubPoseStamped_;
  ros::Publisher pubNavSatFix_;
  ros::Publisher pubPoint_;

  CallbackHandler(ros::NodeHandle& nh) {
    pubPoseWithCovarianceStamped_ =
        nh.advertise<geometry_msgs::PoseWithCovarianceStamped>
        ("pose_with_covariance_output", 100);
    pubTransformStamped_ =
        nh.advertise<geometry_msgs::TransformStamped>("transform_output", 100);
    pubPoseStamped_ =
        nh.advertise<geometry_msgs::PoseStamped>("pose_output", 100);
    pubNavSatFix_ =
        nh.advertise<sensor_msgs::NavSatFix>("navsatfix_output", 100);
    pubPoint_ =
        nh.advertise<geometry_msgs::PointStamped>("point_output", 100);
  }

  void config(Config_T &config, uint32_t /*level*/) {
    config_ = config;
  }

  void MeasurementCallback(
      const geometry_msgs::PoseWithCovarianceStampedConstPtr & msg) {
    if (config_.publish_pose) {
      pubPoseWithCovarianceStamped_.publish(msg);
    }
  }

  void MeasurementCallback(
      const geometry_msgs::TransformStampedConstPtr & msg) {
    if (config_.publish_pose) {
      pubTransformStamped_.publish(msg);
    }
  }

  void MeasurementCallback(const geometry_msgs::PoseStampedConstPtr & msg) {
    if (config_.publish_pose) {
      pubPoseStamped_.publish(msg);
    }
  }

  void MeasurementCallback(const sensor_msgs::NavSatFixConstPtr & msg) {
    if (config_.publish_position) {
      pubNavSatFix_.publish(msg);
    }
  }
  void MeasurementCallback(const geometry_msgs::PointStampedConstPtr & msg) {
    if (config_.publish_position) {
      pubPoint_.publish(msg);
    }
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "msf_distort");

  typedef msf_distort::MSF_DistortConfig Config_T;
  typedef dynamic_reconfigure::Server<Config_T> ReconfigureServer;
  typedef boost::shared_ptr<ReconfigureServer> ReconfigureServerPtr;

  ros::NodeHandle nh("msf_distort");

  CallbackHandler handler(nh);
  ReconfigureServerPtr reconf_server_;  ///< dynamic reconfigure server

  reconf_server_.reset(new ReconfigureServer(nh));
  ReconfigureServer::CallbackType f = boost::bind(&CallbackHandler::config,
                                                  &handler, _1, _2);
  reconf_server_->setCallback(f);

  ros::Subscriber subPoseWithCovarianceStamped_ =
      nh.subscribe < geometry_msgs::PoseWithCovarianceStamped
          > ("pose_with_covariance_input", 20, &CallbackHandler::MeasurementCallback, &handler);
  ros::Subscriber subTransformStamped_ =
      nh.subscribe < geometry_msgs::TransformStamped
          > ("transform_input", 20, &CallbackHandler::MeasurementCallback, &handler);
  ros::Subscriber subPoseStamped_ = nh.subscribe < geometry_msgs::PoseStamped
      > ("pose_input", 20, &CallbackHandler::MeasurementCallback, &handler);
  ros::Subscriber subNavSatFix_ =
      nh.subscribe < sensor_msgs::NavSatFix
          > ("navsatfix_input", 20, &CallbackHandler::MeasurementCallback, &handler);
  ros::Subscriber subPoint_ = nh.subscribe < geometry_msgs::PointStamped
      > ("point_input", 20, &CallbackHandler::MeasurementCallback, &handler);

  ros::V_string topics;
  ros::this_node::getSubscribedTopics(topics);
  std::string nodeName = ros::this_node::getName();
  std::string topicsStr = nodeName + ":\n\tsubscribed to topics:\n";
  for (unsigned int i = 0; i < topics.size(); i++)
    topicsStr += ("\t\t" + topics.at(i) + "\n");

  topicsStr += "\tadvertised topics:\n";
  ros::this_node::getAdvertisedTopics(topics);
  for (unsigned int i = 0; i < topics.size(); i++)
    topicsStr += ("\t\t" + topics.at(i) + "\n");

  MSF_INFO_STREAM("" << topicsStr);

  ros::spin();

}
