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
#include <msf_core/similaritytransform.h>

#include <Eigen/Eigenvalues>

namespace msf_core {
namespace similarity_transform {
From6DoF::From6DoF() { }

void From6DoF::AddMeasurement(const PosePair & measurement) {
  measurements_.push_back(measurement);
}

void From6DoF::AddMeasurement(const Pose & pose1, const Pose & pose2) {
  AddMeasurement(PosePair(pose1, pose2));
}

bool From6DoF::Compute(Pose & pose, double *scale, double *cond, double eps) {
  const int n = 4;  // Number of parameters we need to optimize.
  const int m = measurements_.size();

  if (m < 2)
    return false;

  Matrix4 M(Matrix4::Zero());  // Quaternion outer sum matrix.
  // Matrix collecting the measurements for position and scale.
  Eigen::Matrix<double, Eigen::Dynamic, n> A;
  A.resize(m * 3, Eigen::NoChange);

  Eigen::Matrix<double, Eigen::Dynamic, 1> b;
  b.resize(m * 3, Eigen::NoChange);

  for (int i = 0; i < m; i++) {
    // Quaternion averaging.
    const PosePair & pp = measurements_[i];
    const Eigen::Quaterniond q1 = GeometryMsgsToEigen(
        pp.first.pose.orientation);
    const Eigen::Quaterniond q2 = GeometryMsgsToEigen(
        pp.second.pose.orientation);
    Eigen::Quaterniond q(q1.inverse() * q2);
    M += q.coeffs() * q.coeffs().transpose();  // Order is x y z w here !!!

        //
    const Vector3 t1 = GeometryMsgsToEigen(pp.first.pose.position);
    const Vector3 t2 = GeometryMsgsToEigen(pp.second.pose.position);

    A.block<3, 3>(i * 3, 0) = Eigen::Matrix<double, 3, 3>::Identity() * -1;
    A.block<3, 1>(i * 3, 3) = q1.inverse() * t2;
    b.block<3, 1>(i * 3, 0) = q1.inverse() * t1;
  }

  // Mean quaternion.
  Eigen::SelfAdjointEigenSolver < Eigen::Matrix<double, 4, 4> > q_solver(M);
  // Eigenvalues/vectors are sorted in increasing order here ...
  Eigen::Quaterniond q_mean = Eigen::Quaterniond(
      q_solver.eigenvectors().col(3));

  // Mean position and scale.
  Matrix4 A_hat = A.transpose() * A;
  Matrix4 S_hat(Matrix4::Zero());
  Vector4 b_hat = A.transpose() * b;

  Eigen::JacobiSVD<Matrix4> svd(A_hat,
                                Eigen::ComputeFullU | Eigen::ComputeFullV);
  for (int i = 0; i < n; i++) {
    if (svd.singularValues()[i] < eps)
      S_hat(i, i) = 0;
    else
      S_hat(i, i) = 1 / svd.singularValues()[i];
  }
  if (cond)
    *cond = svd.singularValues()[0] / svd.singularValues()[n - 1];

  Vector4 x = svd.matrixV() * S_hat * svd.matrixU().transpose() * b_hat;
  if (scale)
    *scale = x[3];

  pose.pose.position = EigenToGeometryMsgs(x.block<3, 1>(0, 0));
  pose.pose.orientation = EigenToGeometryMsgs(q_mean);

  return true;
}
}
}  // namespace msf_core
