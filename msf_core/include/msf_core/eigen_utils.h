/*
 * Copyright (C) 2011-2012 Stephan Weiss, ASL, ETH Zurich, Switzerland
 * You can contact the author at <stephan dot weiss at ieee dot org>
 * Copyright (C) 2012-2013 Simon Lynen, ASL, ETH Zurich, Switzerland
 * You can contact the author at <slynen at ethz dot ch>
 * Copyright (c) 2012, Markus Achtelik, ASL, ETH Zurich, Switzerland
 * You can contact the author at <acmarkus at ethz dot ch>
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
#ifndef EIGEN_UTILS_H_
#define EIGEN_UTILS_H_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>

/// Returns the 3D cross product Skew symmetric matrix of a given 3D vector.
template<class Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> Skew(
    const Eigen::MatrixBase<Derived> & vec) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
  return (Eigen::Matrix<typename Derived::Scalar, 3, 3>() << 0.0, -vec[2], vec[1],
      vec[2], 0.0, -vec[0], -vec[1], vec[0], 0.0).finished();
}

/// Returns a matrix with angular velocities used for quaternion derivatives/
// integration with the JPL notation.
/**
 The quaternion to be multiplied with this matrix has to be in the order x y z w !!!
 \param vec 3D vector with angular velocities.
 \return 4x4 matrix for multiplication with the quaternion.
 */
template<class Derived>
inline Eigen::Matrix<typename Derived::Scalar, 4, 4> OmegaMatJPL(
    const Eigen::MatrixBase<Derived> & vec) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
  return (Eigen::Matrix<typename Derived::Scalar, 4, 4>() << 0, vec[2], -vec[1],
      vec[0], -vec[2], 0, vec[0], vec[1], vec[1], -vec[0], 0, vec[2], -vec[0],
      -vec[1], -vec[2], 0)
      .finished();
}

/// Returns a matrix with angular velocities used for quaternion derivatives/
// integration with the Hamilton notation.
/**
 The quaternion to be multiplied with this matrix has to be in the order x y z w !!!
 \param vec 3D vector with angular velocities
 \return 4x4 Matrix for multiplication with the quaternion
 */
template<class Derived>
inline Eigen::Matrix<typename Derived::Scalar, 4, 4> OmegaMatHamilton(
    const Eigen::MatrixBase<Derived> & vec) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
  return (Eigen::Matrix<typename Derived::Scalar, 4, 4>() << 0, -vec[2], vec[1],
      vec[0], vec[2], 0, -vec[0], vec[1], -vec[1], vec[0], 0, vec[2], -vec[0],
      -vec[1], -vec[2], 0).finished();
}

/// Returns a matrix to compute error quaternions
/**
 \param q_vec 4D vector containing the quaternion's coefficients in the order x y z w.
 \return 4x3 matrix for error quaternion computation
 \verbatim
 Xi = [ w*I + Skew([x y z])]  ; dq = [ Xi q]^T * q
 [     -[x y z]       ]
 \endverbatim
 */
template<class Derived>
inline Eigen::Matrix<typename Derived::Scalar, 4, 3> XiMat(
    const Eigen::MatrixBase<Derived> & q_vec) {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, 4, 1);

  return (Eigen::Matrix<typename Derived::Scalar, 4, 3>() <<
  // This is the Xi matrix ---
      q_vec[3] * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity()
          + Skew(q_vec.template head<3>()), -q_vec.template head<3>().transpose()
  // ---
      ).finished();
    }

/// Computes a quaternion from the 3-element small angle approximation theta.
template<class Derived>
Eigen::Quaternion<typename Derived::Scalar> QuaternionFromSmallAngle(
    const Eigen::MatrixBase<Derived> & theta) {
  typedef typename Derived::Scalar Scalar;
  EIGEN_STATIC_ASSERT_FIXED_SIZE (Derived);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
  const Scalar q_squared = theta.squaredNorm() / 4.0;

  if (q_squared < 1) {
    return Eigen::Quaternion < Scalar
        > (sqrt(1 - q_squared), theta[0] * 0.5, theta[1] * 0.5, theta[2] * 0.5);
  } else {
    const Scalar w = 1.0 / sqrt(1 + q_squared);
    const Scalar f = w * 0.5;
    return Eigen::Quaternion < Scalar
        > (w, theta[0] * f, theta[1] * f, theta[2] * f);
  }
}

/// Debug output to check numeric value of Eigen types.
template<class D>
bool CheckForNumeric(const Eigen::MatrixBase<D> & mat,
                     const std::string & info) {
  enum {
    rows = Eigen::MatrixBase < D > ::RowsAtCompileTime,
    cols = Eigen::MatrixBase < D > ::ColsAtCompileTime
  };

  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      if (std::isnan(mat(i, j))) {
        std::cerr << "=== ERROR ===  " << info << ": NAN at index [" << i << ","
            << j << "]" << std::endl;
        return false;
      }
      if (std::isinf(mat(i, j))) {
        std::cerr << "=== ERROR ===  " << info << ": INF at index [" << i << ","
            << j << "]" << std::endl;
        return false;
      }
    }
  }
  return true;
}

#endif  // EIGEN_UTILS_H_
