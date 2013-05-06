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

#include <msf_core/msf_types.tpp>
#include <msf_core/eigen_utils.h>

/// defines the start row and col for the covariance entries in geometry_msgs::PoseWithCovariance
namespace geometry_msgs {
namespace cov {
enum {
  p = 0,
  q = 3
};
}
}

namespace msf_core {

/// converts a geometry_msgs::Point to an Eigen Vector3d
inline Vector3 geometry_msgsToEigen(const geometry_msgs::Point & p) {
  return (Vector3() << p.x, p.y, p.z).finished();
}

/// converts a geometry_msgs::Quaternion to an Eigen::Quaterniond
inline Eigen::Quaterniond geometry_msgsToEigen(
    const geometry_msgs::Quaternion & q) {
  return Eigen::Quaterniond(q.w, q.x, q.y, q.z);
}

/**
 * \brief returns a 3x3 covariance block entry from a geometry_msgs::PoseWithCovariance::covariance array
 * \param gcov the geometry_msgs::PoseWithCovariance::covariance array to get the block from
 * \param start_row start row of the block; use \code geometry_msgs::cov::p and geometry_msgs::cov::q \endcode
 * \param start_col start column of the block; use \code geometry_msgs::cov::p and geometry_msgs::cov::q \endcode
 * \return 3x3 Eigen::Matrix covariance block
 */
inline Matrix3 geometry_msgsCovBlockToEigen(
    const geometry_msgs::PoseWithCovariance::_covariance_type & gcov,
    int start_row, int start_col) {
  Eigen::Map<const Matrix6> cov(gcov.data());
  return Matrix3(cov.block<3, 3>(start_row, start_col));
}

/**
 * \brief fills a 3x3 covariance block entry of a geometry_msgs::PoseWithCovariance::covariance array from an Eigen 3x3 matrix
 * \param[out] gcov the geometry_msgs::PoseWithCovariance::covariance array to get the block from
 * \param[in] ecov 3x3 Eigen::Matrix covariance block
 * \param[in] start_row start row of the block; use \code geometry_msgs::cov::p and geometry_msgs::cov::q \endcode
 * \param[in] start_col start column of the block; use \code geometry_msgs::cov::p and geometry_msgs::cov::q \endcode
 */
template<class Derived>
inline void eigenCovBlockToGeometry_msgs(
    geometry_msgs::PoseWithCovariance::_covariance_type & gcov,
    const Eigen::MatrixBase<Derived> &ecov, int start_row, int start_col) {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, 3, 3)
  Eigen::Map<Matrix6> _gcov(gcov.data());
  _gcov.block<3, 3>(start_row, start_col) = ecov;
  if (start_row != start_col)  // off diagonal entries --> we need to copy it to the opposite off-diagonal as well
    _gcov.block<3, 3>(start_col, start_row) = ecov.transpose();
}

/// converts any eigen vector with 3 elements to a geometry_msgs::Point
template<class Derived>
inline geometry_msgs::Point eigenToGeometry_msgs(
    const Eigen::MatrixBase<Derived> & p) {
  EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived);
  assert(p.size()==3);

  geometry_msgs::Point _p;
  _p.x = p[0];
  _p.y = p[1];
  _p.z = p[2];
  return _p;
}

/// converts an Eigen::Quaterniond to a geometry_msgs::Quaternion
inline geometry_msgs::Quaternion eigenToGeometry_msgs(
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
 * \brief computes the average similarity transform (rotation, position, scale) between two sets of 6 DoF poses
 */
class From6DoF {
 public:
  From6DoF();
  /// Adds a pair of measurements. No other computation is performed
  void addMeasurement(const PosePair & measurement);

  /// Adds a pair of measurements. No other computation is performed
  void addMeasurement(const Pose & pose1, const Pose & pose2);

  /**
   * \brief computes the average similarity transform
   * the transform is computed between the previously added sets of poses
   * \param[out] pose resulting position / orientation
   * \param[out] scale resulting scale: pos1 = pos2*scale
   * \param[out] cond condition number of the problem to detect (non) sufficient number of measurements
   * \param[in] eps in case of badly posed problems, eps can be set for the Moore-Penrose pseudo inverse
   */
  bool compute(Pose & pose, double *scale = NULL, double *cond = NULL,
               double eps = std::numeric_limits<double>::epsilon() * 4 * 4);
 private:
  PosePairVector measurements_;
};
}
} /* namespace msf_core */
#endif /* SIMILARITYTRANSFORM_H_ */
