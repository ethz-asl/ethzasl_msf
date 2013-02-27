/*
 * test_distort.cc
 *
 *  Created on: Feb 27, 2013
 *      Author: slynen
 */

#include <msf_updates/PoseDistorter.h>
#include <iostream>

int main(int argc, char** argv){

  Eigen::Vector3d meanpos;
  meanpos.setConstant(0.01);

  Eigen::Vector3d stddevpos;
  stddevpos.setConstant(0.001);

  Eigen::Vector3d meanatt;
  meanatt.setConstant(0.01);

  Eigen::Vector3d stddevatt;
  stddevatt.setConstant(0.001);

  double meanscaledrift = 0.001;
  double stddevscaledrift = 0.0001;

  msf_updates::PoseDistorter dist(meanpos, stddevpos, meanatt, stddevatt, meanscaledrift, stddevscaledrift);

  Eigen::Vector3d pos;
  pos.setZero();
  Eigen::Quaterniond att;
  att.setIdentity();

  std::cout<<"distortions"<<std::endl;
  double dt = 1.0 / 20;
  for(int i = 0;i<10;++i){
    dist.distort(pos, dt);
    dist.distort(att, dt);
    std::cout<<"pos "<<i<<": ["<<pos.transpose()<<"]"<<std::endl;
    std::cout<<"att "<<i<<": ["<<att.w()<<", "<<att.x()<<", "<<att.y()<<", "<<att.z()<<"]"<<std::endl<<std::endl;
  }

}
