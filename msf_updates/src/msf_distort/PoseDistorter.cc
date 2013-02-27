/*
 * PoseDistorter.cc
 *
 *  Created on: Feb 27, 2013
 *      Author: slynen
 */

#include <msf_updates/PoseDistorter.h>
#include <msf_core/eigen_utils.h>

namespace msf_updates
{

PoseDistorter::PoseDistorter(const Eigen::Vector3d& meanposdrift, const Eigen::Vector3d& stddevposdrift, const Eigen::Vector3d& meanattdrift, const Eigen::Vector3d& stddevattdrift):
                meanposdrift_(meanposdrift), stddevposdrift_(stddevposdrift), meanattdrift_(meanattdrift), sigmaattdrift_(stddevattdrift), gen_(rd_())
{
  posdrift_.setZero();
  attdrift_.setIdentity();
  for(int i = 0 ; i<3 ; ++i){
    d_pos_[i] = distribution_t(meanposdrift(i), stddevposdrift(i));
    d_att_[i] = distribution_t(meanattdrift(i), stddevattdrift(i));
  }
}

PoseDistorter::~PoseDistorter()
{

}

void PoseDistorter::distort(Eigen::Vector3d& pos, double dt){
  //calculate distortions
  Eigen::Vector3d deltapos;
  deltapos << d_pos_[0](gen_) * dt, d_pos_[1](gen_) * dt, d_pos_[2](gen_) * dt;

  //add to rand walk
  posdrift_ += deltapos;

  //augment state
  pos += posdrift_;
}

void PoseDistorter::distort(Eigen::Quaterniond& att, double dt){
  //calculate distortions
  Eigen::Vector3d rpydist;
  rpydist << d_att_[0](gen_) * dt, d_att_[1](gen_) * dt, d_att_[2](gen_) * dt;
  Eigen::Quaterniond deltaquat = quaternionFromSmallAngle(rpydist);

  deltaquat.normalize();

  //add to rand walk
  attdrift_ *= deltaquat;

  //augment state
  att *= attdrift_;

}

void PoseDistorter::distort(Eigen::Vector3d& pos, Eigen::Quaterniond& att, double dt){
  distort(pos, dt);
  distort(att, dt);
}

} /* namespace msf_updates */
