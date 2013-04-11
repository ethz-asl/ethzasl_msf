
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <msf_distort/MSF_DistortConfig.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/server.h>
#include <Eigen/Dense>
#include <queue>


struct CallbackHandler{

  typedef msf_distort::MSF_DistortConfig Config_T;
  Config_T config_;
  ros::Publisher pubPoseWithCovarianceStamped_;
  ros::Publisher pubTransformStamped_;
  ros::Publisher pubPoseStamped_;
  ros::Publisher pubNavSatFix_;
  ros::Publisher pubPoint_;
  std::queue<geometry_msgs::TransformStampedConstPtr> queue;

  CallbackHandler(ros::NodeHandle& nh){
    pubPoseWithCovarianceStamped_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped> ("pose_with_covariance_output", 100);
    pubTransformStamped_ = nh.advertise<geometry_msgs::TransformStamped> ("transform_output", 100);
    pubPoseStamped_ = nh.advertise<geometry_msgs::PoseStamped> ("pose_output", 100);
    pubNavSatFix_ = nh.advertise<sensor_msgs::NavSatFix>("navsatfix_output", 100);
    pubPoint_ = nh.advertise<geometry_msgs::PointStamped>("point_output", 100);
  }

  void config(Config_T &config, uint32_t level){
    config_ = config;
  }

  void measurementCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr & msg){
    if(config_.publish_pose){
      pubPoseWithCovarianceStamped_.publish(msg);
    }
  }

  void measurementCallback(const geometry_msgs::TransformStampedConstPtr & newmsg){
    if(config_.publish_pose){
      pubTransformStamped_.publish(newmsg);
    }
    double delay = 0.0;
    queue.push(newmsg);

    double current_time = newmsg->header.stamp.toSec();

    if(queue.front()->header.stamp.toSec() <= current_time - delay){
      geometry_msgs::TransformStampedConstPtr msg = queue.front();
      queue.pop();

      geometry_msgs::PoseStampedPtr msgpose(new geometry_msgs::PoseStamped);
      msgpose->header = msg->header;
      msgpose->header.stamp = ros::Time(msgpose->header.stamp.toSec());
      msgpose->pose.position.x = 0;//msg->transform.translation.x;
      msgpose->pose.position.y = 0;//msg->transform.translation.y;
      msgpose->pose.position.z = 0;//msg->transform.translation.z;

      //calibration camera body to vicon marker body
      Eigen::Quaterniond align(0.0041,0.0056,-0.0315,-0.9995);
      align.normalize();
      Eigen::Quaterniond cam(0,1,0,0);
      cam.normalize();
      Eigen::Quaterniond rotz(0,0,0,1);
      rotz.normalize();

      Eigen::Quaterniond gt_pose(msg->transform.rotation.w, msg->transform.rotation.x, msg->transform.rotation.y, msg->transform.rotation.z);
      Eigen::Quaterniond aligned = align.inverse() * gt_pose;

      msgpose->pose.orientation.w = aligned.w();
      msgpose->pose.orientation.x = aligned.x();
      msgpose->pose.orientation.y = aligned.y();
      msgpose->pose.orientation.z = aligned.z();
      pubPoseStamped_.publish(msgpose);

    }


  }

  void measurementCallback(const geometry_msgs::PoseStampedConstPtr & msg){
    if(config_.publish_pose){
      pubPoseStamped_.publish(msg);
    }
  }

  void measurementCallback(const sensor_msgs::NavSatFixConstPtr & msg){
    if(config_.publish_position){
      pubNavSatFix_.publish(msg);
    }
  }
  void measurementCallback(const geometry_msgs::PointStampedConstPtr & msg){
    if(config_.publish_position){
      pubPoint_.publish(msg);
    }
  }
};

int main(int argc, char** argv){

  ros::init(argc, argv, "msf_distort");

  typedef msf_distort::MSF_DistortConfig Config_T;
  typedef dynamic_reconfigure::Server<Config_T> ReconfigureServer;
  typedef boost::shared_ptr<ReconfigureServer> ReconfigureServerPtr;

  ros::NodeHandle nh("msf_distort");

  CallbackHandler handler(nh);
  ReconfigureServerPtr reconf_server_; ///< dynamic reconfigure server

  reconf_server_.reset(new ReconfigureServer(nh));
  ReconfigureServer::CallbackType f = boost::bind(&CallbackHandler::config, &handler, _1, _2);
  reconf_server_->setCallback(f);

  ros::Subscriber subPoseWithCovarianceStamped_ = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("pose_with_covariance_input", 20, &CallbackHandler::measurementCallback, &handler);
  ros::Subscriber subTransformStamped_ = nh.subscribe<geometry_msgs::TransformStamped>("transform_input", 20, &CallbackHandler::measurementCallback, &handler);
  ros::Subscriber subPoseStamped_ = nh.subscribe<geometry_msgs::PoseStamped>("pose_input", 20, &CallbackHandler::measurementCallback, &handler);
  ros::Subscriber subNavSatFix_ = nh.subscribe<sensor_msgs::NavSatFix>("navsatfix_input", 20, &CallbackHandler::measurementCallback, &handler);
  ros::Subscriber subPoint_ = nh.subscribe<geometry_msgs::PointStamped>("point_input", 20, &CallbackHandler::measurementCallback, &handler);

  ros::V_string topics;
  ros::this_node::getSubscribedTopics(topics);
  std::string nodeName = ros::this_node::getName();
  std::string topicsStr = nodeName + ":\n\tsubscribed to topics:\n";
  for(unsigned int i=0; i<topics.size(); i++)
    topicsStr+=("\t\t" + topics.at(i) + "\n");

  topicsStr += "\tadvertised topics:\n";
  ros::this_node::getAdvertisedTopics(topics);
  for(unsigned int i=0; i<topics.size(); i++)
    topicsStr+=("\t\t" + topics.at(i) + "\n");

  ROS_INFO_STREAM(""<< topicsStr);

  ros::spin();

}
