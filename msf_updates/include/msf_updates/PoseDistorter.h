/*
 * PoseDistorter.h
 *
 *  Created on: Feb 27, 2013
 *      Author: slynen
 */

#ifndef POSEDISTORTER_H_
#define POSEDISTORTER_H_

#include <Eigen/Dense>
#include <random>
#include <cmath>

namespace msf_updates
{

class PoseDistorter
{
public:
  typedef std::normal_distribution<> distribution_t;
private:
  Eigen::Vector3d meanposdrift_;
  Eigen::Vector3d stddevposdrift_;
  Eigen::Vector3d meanattdrift_;
  Eigen::Vector3d sigmaattdrift_;
  Eigen::Vector3d posdrift_;
  Eigen::Quaterniond attdrift_;
  std::random_device rd_;
  std::mt19937 gen_;
  distribution_t d_pos_[3];
  distribution_t d_att_[3];
public:
  PoseDistorter(const Eigen::Vector3d& meanposdrift, const Eigen::Vector3d& stddevposdrift, const Eigen::Vector3d& meanattdrift, const Eigen::Vector3d& stddevattdrift);
  void distort(Eigen::Vector3d& pos, Eigen::Quaterniond& att, double dt);
  void distort(Eigen::Vector3d& pos, double dt);
  void distort(Eigen::Quaterniond& att, double dt);
  virtual ~PoseDistorter();
};

} /* namespace msf_updates */
#endif /* POSEDISTORTER_H_ */
