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
#include <msf_core/msf_types.tpp>
#include <msf_core/msf_fwds.h>

namespace msf_updates
{

class PoseDistorter
{
public:
  typedef std::normal_distribution<> distribution_t;
  typedef shared_ptr<PoseDistorter> Ptr;
private:
  Eigen::Vector3d posdrift_;
  Eigen::Quaterniond attdrift_;
  double scaledrift_;

  std::random_device rd_;
  std::mt19937 gen_;

  distribution_t d_pos_[3];
  distribution_t d_att_[3];
  distribution_t d_scale;
public:
  PoseDistorter(const Eigen::Vector3d& meanposdrift, const Eigen::Vector3d& stddevposdrift, const Eigen::Vector3d& meanattdrift, const Eigen::Vector3d& stddevattdrift, const double meanscaledrift, const double stddevscaledrift);
  void distort(Eigen::Vector3d& pos, Eigen::Quaterniond& att, double dt);
  void distort(Eigen::Vector3d& pos, double dt);
  void distort(Eigen::Quaterniond& att, double dt);
  virtual ~PoseDistorter();
};

} /* namespace msf_updates */
#endif /* POSEDISTORTER_H_ */
