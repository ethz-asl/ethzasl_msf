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

#include <ros/ros.h>
#include <msf_core/msf_core.hpp>
int main(int argc, char** argv)
{
//just to instantiate all templates, no production code

//	ros::init(argc, argv, "msf_core");
//	boost::shared_ptr<msf_core::MSF_SensorManager> usercalcs(new msf_core::MSF_InitMeasurement(true));
//	msf_core::MSF_Core core(usercalcs);

	 map<double,int> mymap;
	  map<double,int>::iterator it,itlow,itup;

	  mymap[0.1]=20;
	  mymap[0]=40;
	  mymap[5]=60;
	  mymap[10]=80;
	  mymap[0.01]=100;

	  itlow=mymap.lower_bound ('b');  // itlow points to b
	  itup=mymap.upper_bound ('d');   // itup points to e (not d!)

	  mymap.erase(itlow,itup);        // erases [itlow,itup)

	  // print content:
	  for ( it=mymap.begin() ; it != mymap.end(); it++ )
	    cout << (*it).first << " => " << (*it).second << endl;

	  return 0;

//	ros::spin();
}

