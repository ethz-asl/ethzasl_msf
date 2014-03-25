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
#include <msf_updates/PoseDistorter.h>
#include <iostream>

int main(){

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
    dist.Distort(pos, dt);
    dist.Distort(att, dt);
    std::cout<<"pos "<<i<<": ["<<pos.transpose()<<"]"<<std::endl;
    std::cout<<"att "<<i<<": ["<<att.w()<<", "<<att.x()<<", "<<att.y()<<", "<<att.z()<<"]"<<std::endl<<std::endl;
  }

}
