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

#ifndef EIGEN_UTILS_H_
#define EIGEN_UTILS_H_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>

/// Returns the 3D cross product skew symmetric matrix of a given 3D vector.
template<class Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> skew(
    const Eigen::MatrixBase<Derived> & vec) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
  return (Eigen::Matrix<typename Derived::Scalar, 3, 3>() << 0.0, -vec[2],
      vec[1], vec[2], 0.0, -vec[0], -vec[1], vec[0], 0.0).finished();
}

/// Returns a matrix with angular velocities used for quaternion derivatives/
// integration with the JPL notation.
/**
 The quaternion to be multiplied with this matrix has to be in the order x y z w !!!
 \param vec 3D vector with angular velocities.
 \return 4x4 matrix for multiplication with the quaternion.
 */
template<class Derived>
inline Eigen::Matrix<typename Derived::Scalar, 4, 4> omegaMatJPL(
    const Eigen::MatrixBase<Derived> & vec) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
  return (Eigen::Matrix<typename Derived::Scalar, 4, 4>() << 0, vec[2], -vec[1],
      vec[0], -vec[2], 0, vec[0], vec[1], vec[1], -vec[0], 0, vec[2], -vec[0],
      -vec[1], -vec[2], 0).finished();
}

/// Returns a matrix with angular velocities used for quaternion derivatives/
// integration with the Hamilton notation.
/**
 The quaternion to be multiplied with this matrix has to be in the order x y z w !!!
 \param vec 3D vector with angular velocities
 \return 4x4 Matrix for multiplication with the quaternion
 */
template<class Derived>
inline Eigen::Matrix<typename Derived::Scalar, 4, 4> omegaMatHamilton(
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
 Xi = [ w*I + skew([x y z])]  ; dq = [ Xi q]^T * q
 [     -[x y z]       ]
 \endverbatim
 */
template<class Derived>
inline Eigen::Matrix<typename Derived::Scalar, 4, 3> xiMat(
    const Eigen::MatrixBase<Derived> & q_vec) {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, 4, 1);

  return (Eigen::Matrix<typename Derived::Scalar, 4, 3>() <<
  // This is the Xi matrix ---
      q_vec[3] * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity()
          + skew(q_vec.template head<3>()), -q_vec.template head<3>().transpose()
  // ---
      ).finished();
    }

/// Computes a quaternion from the 3-element small angle approximation theta.
template<class Derived>
Eigen::Quaternion<typename Derived::Scalar> quaternionFromSmallAngle(
    const Eigen::MatrixBase<Derived> & theta) {
  typedef typename Derived::Scalar Scalar;
  EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
  const Scalar q_squared = theta.squaredNorm() / 4.0;

  if (q_squared < 1) {
    return Eigen::Quaternion<Scalar>(sqrt(1 - q_squared), theta[0] * 0.5,
                                     theta[1] * 0.5, theta[2] * 0.5);
  } else {
    const Scalar w = 1.0 / sqrt(1 + q_squared);
    const Scalar f = w * 0.5;
    return Eigen::Quaternion<Scalar>(w, theta[0] * f, theta[1] * f,
                                     theta[2] * f);
  }
}

/// Debug output to check misbehavior of Eigen.
template<class D>
bool checkForNumeric(const Eigen::MatrixBase<D> & mat,
                     const std::string & info) {
  enum {
    rows = Eigen::MatrixBase<D>::RowsAtCompileTime,
    cols = Eigen::MatrixBase<D>::ColsAtCompileTime
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
