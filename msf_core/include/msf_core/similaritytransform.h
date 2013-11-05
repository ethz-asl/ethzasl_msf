/*
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
#ifndef SIMILARITYTRANSFORM_H_
#define SIMILARITYTRANSFORM_H_

#include <vector>
#include <utility>
#include <geometry_msgs/PoseWithCovariance.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <limits>

#include <msf_core/msf_types.h>
#include <msf_core/eigen_utils.h>

/// Defines the start row and col for the covariance entries in
//  geometry_msgs::PoseWithCovariance.
namespace geometry_msgs {
namespace cov {
enum {
  p = 0,
  q = 3
};
}
}

namespace msf_core {

/// Converts a geometry_msgs::Point to an Eigen Vector3d.
inline Vector3 GeometryMsgsToEigen(const geometry_msgs::Point & p) {
  return (Vector3() << p.x, p.y, p.z).finished();
}

/// Converts a geometry_msgs::Quaternion to an Eigen::Quaterniond.
inline Eigen::Quaterniond GeometryMsgsToEigen(
    const geometry_msgs::Quaternion & q) {
  return Eigen::Quaterniond(q.w, q.x, q.y, q.z);
}

/**
 * \brief Returns a 3x3 covariance block entry from a
 * geometry_msgs::PoseWithCovariance::covariance array
 * \param gcov the geometry_msgs::PoseWithCovariance::covariance array to get
 * the block from.
 * \param start_row start row of the block; use \code geometry_msgs::cov::p and
 * geometry_msgs::cov::q \endcode .
 * \param start_col start column of the block; use \code geometry_msgs::cov::p
 * and geometry_msgs::cov::q \endcode .
 * \return 3x3 Eigen::Matrix covariance block.
 */
inline Matrix3 GeometryMsgsCovBlockToEigen(
    const geometry_msgs::PoseWithCovariance::_covariance_type & gcov,
    int start_row, int start_col) {
  Eigen::Map<const Matrix6> cov(gcov.data());
  return Matrix3(cov.block<3, 3>(start_row, start_col));
}

/**
 * \brief Fills a 3x3 covariance block entry of a
 * geometry_msgs::PoseWithCovariance::covariance array from an Eigen 3x3 matrix
 * \param[out] gcov the geometry_msgs::PoseWithCovariance::covariance array to
 * get the block from
 * \param[in] ecov 3x3 Eigen::Matrix covariance block
 * \param[in] start_row start row of the block; use \code geometry_msgs::cov::p
 * and geometry_msgs::cov::q \endcode
 * \param[in] start_col start column of the block; use \code geometry_msgs::cov::p
 * and geometry_msgs::cov::q \endcode
 */
template<class Derived>
inline void EigenCovBlockToGeometryMsgs(
    geometry_msgs::PoseWithCovariance::_covariance_type & gcov,
    const Eigen::MatrixBase<Derived> &ecov, int start_row, int start_col) {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, 3, 3)
  Eigen::Map<Matrix6> _gcov(gcov.data());
  _gcov.block<3, 3>(start_row, start_col) = ecov;
  // Off diagonal entries --> we need to copy it to the opposite off-diagonal
  // as well.
  if (start_row != start_col)
    _gcov.block<3, 3>(start_col, start_row) = ecov.transpose();
}

/// Converts any eigen vector with 3 elements to a geometry_msgs::Point.
template<class Derived>
inline geometry_msgs::Point EigenToGeometryMsgs(
    const Eigen::MatrixBase<Derived> & p) {
  EIGEN_STATIC_ASSERT_VECTOR_ONLY (Derived);
  assert(p.size() == 3);

  geometry_msgs::Point _p;
  _p.x = p[0];
  _p.y = p[1];
  _p.z = p[2];
  return _p;
}

/// Converts an Eigen::Quaterniond to a geometry_msgs::Quaternion.
inline geometry_msgs::Quaternion EigenToGeometryMsgs(
    const Eigen::Quaterniond & q) {
  geometry_msgs::Quaternion _q;
  _q.w = q.w();
  _q.x = q.x();
  _q.y = q.y();
  _q.z = q.z();
  return _q;
}

namespace similarity_transform {

typedef geometry_msgs::PoseWithCovariance Pose;
typedef std::pair<Pose, Pose> PosePair;
typedef std::vector<PosePair> PosePairVector;

/** \class From6DoF
 *
 * \brief Computes the average similarity transform (rotation, position, scale)
 * between two sets of 6 DoF poses.
 */
class From6DoF {
 public:
  From6DoF();
  /// Adds a pair of measurements. No other computation is performed.
  void AddMeasurement(const PosePair & measurement);

  /// Adds a pair of measurements. No other computation is performed.
  void AddMeasurement(const Pose & pose1, const Pose & pose2);

  /**
   * \brief computes the average similarity transform
   * the transform is computed between the previously added sets of poses.
   * \param[out] pose resulting position / orientation
   * \param[out] scale resulting scale: pos1 = pos2*scale
   * \param[out] cond condition number of the problem to detect (non) sufficient
   * number of measurements
   * \param[in] eps in case of badly posed problems, eps can be set for the
   * Moore-Penrose pseudo inverse
   */
  bool Compute(Pose & pose, double *scale = nullptr , double *cond = nullptr ,
               double eps = std::numeric_limits<double>::epsilon() * 4 * 4);
 private:
  PosePairVector measurements_;
};
}
}  // namespace msf_core
#endif  // SIMILARITYTRANSFORM_H_
