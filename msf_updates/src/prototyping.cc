
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <random>
#include <assert.h>
#include <cstdlib>
#include <msf_core/msf_core.hpp>

int main(int argc, char** argv)
{

	msf_core::MSF_Core::stateBufferT buff;

	for(int i = 0;i<10;++i){
		msf_core::EKFStatePtr state(new msf_core::EKFState);
		state->time_ = 100. + i/6. + ((5+i)%(i+1)) / 7;
		buff.insert(state);
	}

	std::cout<<"buffer"<<std::endl;
	std::cout<<buff.echoBufferContentTimes();

	double reqtime = 100.1;
	std::cout<<"closest to "<<reqtime<<" "<<buff.getClosest(reqtime)->time_<<std::endl;
	reqtime = 100.101;
	std::cout<<"closest to "<<reqtime<<" "<<buff.getClosest(reqtime)->time_<<std::endl;
	reqtime = 100.231;
	std::cout<<"closest to "<<reqtime<<" "<<buff.getClosest(reqtime)->time_<<std::endl;
	reqtime = 100.151;
	std::cout<<"closest to "<<reqtime<<" "<<buff.getClosest(reqtime)->time_<<std::endl;
	reqtime = 100.371;
	std::cout<<"closest to "<<reqtime<<" "<<buff.getClosest(reqtime)->time_<<std::endl;
	reqtime = 100.456;
	std::cout<<"closest to "<<reqtime<<" "<<buff.getClosest(reqtime)->time_<<std::endl;
	reqtime = 100.123;
	std::cout<<"closest to "<<reqtime<<" "<<buff.getClosest(reqtime)->time_<<std::endl;
	reqtime = 100.567;
	std::cout<<"closest to "<<reqtime<<" "<<buff.getClosest(reqtime)->time_<<std::endl;

}
