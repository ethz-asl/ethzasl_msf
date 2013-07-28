// Bring in gtest
#include <gtest/gtest.h>
#include <msf_simulate/MSFSimulator.h>
#include <msf_simulate/pose_sensormanager.h>

//to help eclipse get things right
#ifndef TEST
#define TEST(a,b) void func_##a_##b()
#endif

TEST(msf_testsuite, testTrajectory)
{

  msf_simulate::MSFSimulatorOptions options;
  msf_simulate::MSFSimulator simulator(options);

  Vertex1D cp(1,1);
  cp.addConstraint(1,3);

  simulator.updatePathWithConstraint(DerivativesP::s, cp, cp);

  msf_simulate::MSFSimulator::MotionVector states;

  simulator.getMotion(states);

  std::cout<<"Got "<<states.size()<<" states"<<std::endl;


  msf_pose_sensor::PoseSensorManager manager;


}
