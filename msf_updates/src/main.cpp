/*

Copyright (c) 2010, Stephan Weiss, ASL, ETH Zurich, Switzerland
You can contact the author at <stephan dot weiss at ieee dot org>

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

#ifdef POSE_MEAS
#include "pose_measurements.h"
#endif
#ifdef POSITION_MEAS
#include "position_measurements.h"
#endif

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ssf_core");
#ifdef POSE_MEAS
	PoseMeasurements PoseMeas;
	ROS_INFO_STREAM("Filter type: pose_sensor");
#endif

#ifdef POSITION_MEAS
	PositionMeasurements PositionMeas;
	ROS_INFO_STREAM("Filter type: position_sensor");
#endif


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
