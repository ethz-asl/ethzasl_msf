/*
 * Copyright (C) 2011-2012 Stephan Weiss, ASL, ETH Zurich, Switzerland
 * You can contact the author at <stephan dot weiss at ieee dot org>
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
#ifndef EIGEN_CONVERSIONS_H_
#define EIGEN_CONVERSIONS_H_

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>

namespace eigen_conversions {
/// Copies eigen quaternion to geometry_msgs/quaternion.
template<class Scalar>
inline void QuaternionToMsg(const Eigen::Quaternion<Scalar>& q_in,
                            geometry_msgs::Quaternion& q_out) {
  q_out.w = q_in.w();
  q_out.x = q_in.x();
  q_out.y = q_in.y();
  q_out.z = q_in.z();
}

/// Copies eigen quaternion to geometry_msgs/quaternion.
template<class Scalar>
inline geometry_msgs::Quaternion QuaternionToMsg(
    const Eigen::Quaternion<Scalar>& q_in) {
  geometry_msgs::Quaternion q_out;
  QuaternionToMsg(q_in, q_out);
  return q_out;
}

/// Copies an eigen 3d vector to a 3d Point struct. point has to have members
/// x,y,z!
template<class Derived, class Point>
inline void Vector3dToPoint(const Eigen::MatrixBase<Derived>& vec,
                            Point& point) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
  point.x = vec[0];
  point.y = vec[1];
  point.z = vec[2];
}

/// Copies an eigen 3d vector to a 3d Point struct. point has to have members
/// x,y,z!
template<class Derived, class Point>
inline Point Vector3dToPoint(const Eigen::MatrixBase<Derived>& vec) {
  Point point;
  Vector3dToPoint(vec, point);
  return point;
}
}
#endif  // EIGEN_CONVERSIONS_H_
