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

#ifndef EIGEN_CONVERSIONS_H_
#define EIGEN_CONVERSIONS_H_

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>

namespace eigen_conversions
{

/// copies eigen quaternion to geometry_msgs/quaternion
template<class Scalar>
  inline void quaternionToMsg(const Eigen::Quaternion<Scalar> & q_in, geometry_msgs::Quaternion & q_out)
  {
    q_out.w = q_in.w();
    q_out.x = q_in.x();
    q_out.y = q_in.y();
    q_out.z = q_in.z();
  }

/// copies eigen quaternion to geometry_msgs/quaternion
template<class Scalar>
  inline geometry_msgs::Quaternion quaternionToMsg(const Eigen::Quaternion<Scalar> & q_in)
  {
    geometry_msgs::Quaternion q_out;
    quaternionToMsg(q_in, q_out);
    return q_out;
  }

/// copies an eigen 3d vector to a 3d Point struct. point has to have members x,y,z!
template<class Derived, class Point>
  inline void vector3dToPoint(const Eigen::MatrixBase<Derived> & vec, Point & point)
  {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
    point.x = vec[0];
    point.y = vec[1];
    point.z = vec[2];
  }

/// copies an eigen 3d vector to a 3d Point struct. point has to have members x,y,z!
template<class Derived, class Point>
  inline Point vector3dToPoint(const Eigen::MatrixBase<Derived> & vec)
  {
    Point point;
    vector3dToPoint(vec, point);
    return point;
  }

}
;

#endif /* EIGEN_CONVERSIONS_H_ */
