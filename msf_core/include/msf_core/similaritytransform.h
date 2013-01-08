/*

Copyright (c) 2013, Markus Achtelik, ASL, ETH Zurich, Switzerland
You can contact the author at <markus dot achtelik at mavt dot ethz dot ch>

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


#ifndef SIMILARITYTRANSFORM_H_
#define SIMILARITYTRANSFORM_H_

#include <vector>
#include <utility>
#include <geometry_msgs/PoseWithCovariance.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <limits>

namespace msf_core
{


inline Eigen::Vector3d geometry_msgsToEigen(const geometry_msgs::Point & p)
{
  return (Eigen::Vector3d() << p.x, p.y, p.z).finished();
}

inline Eigen::Quaterniond geometry_msgsToEigen(const geometry_msgs::Quaternion & q)
{
  return Eigen::Quaterniond(q.w, q.x, q.y, q.z);
}

template<class Derived>
  inline geometry_msgs::Point eigenToGeometry_msgs(const Eigen::MatrixBase<Derived> & p)
  {
    EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived);
    assert(p.size()==3);

    geometry_msgs::Point _p;
    _p.x = p[0];
    _p.y = p[1];
    _p.z = p[2];
    return _p;
  }

inline geometry_msgs::Quaternion eigenToGeometry_msgs(const Eigen::Quaterniond & q)
{
  geometry_msgs::Quaternion _q;
  _q.w = q.w();
  _q.x = q.x();
  _q.y = q.y();
  _q.z = q.z();
  return _q;
}

namespace similarity_transform
{

typedef geometry_msgs::PoseWithCovariance Pose;
typedef std::pair<Pose, Pose> PosePair;
typedef std::vector<PosePair> PosePairVector;
class From6DoF
{
public:
  From6DoF();
  void addMeasurement(const PosePair & measurement);
  void addMeasurement(const Pose & pose1, const Pose & pose2);
  bool compute(Pose & pose, double *scale=NULL, double *cond=NULL, double eps=std::numeric_limits<double>::epsilon()*4*4);
private:
  PosePairVector measurements_;
};
}
} /* namespace msf_core */
#endif /* SIMILARITYTRANSFORM_H_ */
