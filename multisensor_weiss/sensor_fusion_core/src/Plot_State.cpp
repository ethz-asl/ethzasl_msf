/*
 * Plot_State.cpp
 *
 *  Created on: Sep 30, 2010
 *      Author: sweiss
 */

#include <ros/ros.h>
#include <sensor_fusion_core/DoubleArrayStamped.h>


void plot(const sensor_fusion_core::DoubleArrayStampedConstPtr & msg);

int main(int argc, char** argv)
{

	ros::init(argc, argv, "plot_state");
	ros::NodeHandle nh("plot_state");
	ros::Subscriber subState_ = nh.subscribe("filter_state", 1, &plot);



	//  print published/subscribed topics
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

	return 0;
}

void plot(const sensor_fusion_core::DoubleArrayStampedConstPtr & msg)
{

	printf("do the plotting here.... \n");
}
