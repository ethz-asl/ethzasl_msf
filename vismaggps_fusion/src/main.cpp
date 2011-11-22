/*
 * main.cpp
 *
 *  Created on: Sep 30, 2010
 *      Author: sweiss
 */

#include "vismaggps_measurements.h"

#include <vector>
#include <utility>

int main(int argc, char** argv)
{

	ros::init(argc, argv, "vismaggps_fusion");
	VisMagGPSMeasurements vismaggpsMeas;
	ROS_INFO_STREAM("Filter type: vismaggps_fusion");

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
