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

#include <msf_core/similaritytransform.h>

#include <Eigen/Eigenvalues>

namespace msf_core
{
namespace similarity_transform
{
From6DoF::From6DoF()
{

}

void From6DoF::addMeasurement(const PosePair & measurement)
{
  measurements_.push_back(measurement);
}

void From6DoF::addMeasurement(const Pose & pose1, const Pose & pose2)
{
  addMeasurement(PosePair(pose1, pose2));
}

bool From6DoF::compute(Pose & pose, double *scale, double *cond, double eps)
{
  const int n = 4; // number of parameters we need to optimize
  typedef Eigen::Matrix<double, n, n> Matrix4d;
  typedef Eigen::Matrix<double, n, 1> Vector4d;
  const int m = measurements_.size();

  if (m < 2)
    return false;


  Matrix4d M(Matrix4d::Zero()); // quaternion outer sum matrix
  Eigen::Matrix<double, Eigen::Dynamic, n> A; // matrix collecting the measurements for position and scale
  A.resize(m * 3, Eigen::NoChange);

  Eigen::Matrix<double, Eigen::Dynamic, 1> b;
  b.resize(m * 3, Eigen::NoChange);

  for (int i = 0; i < m; i++)
  {
    // quaternion averaging
    const PosePair & pp = measurements_[i];
    const Eigen::Quaterniond q1 = geometry_msgsToEigen(pp.first.pose.orientation);
    const Eigen::Quaterniond q2 = geometry_msgsToEigen(pp.second.pose.orientation);
    Eigen::Quaterniond q(q1.inverse() * q2);
    M += q.coeffs() * q.coeffs().transpose(); // order is x y z w here !!!

        //
    const Eigen::Vector3d t1 = geometry_msgsToEigen(pp.first.pose.position);
    const Eigen::Vector3d t2 = geometry_msgsToEigen(pp.second.pose.position);

    A.block<3, 3>(i * 3, 0) = Eigen::Matrix<double, 3, 3>::Identity() * -1;
    A.block<3, 1>(i * 3, 3) = q1.inverse() * t2;
    b.block<3, 1>(i * 3, 0) = q1.inverse() * t1;
  }

  // mean quaternion
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 4, 4> > q_solver(M);
  Eigen::Quaterniond q_mean = Eigen::Quaterniond(q_solver.eigenvectors().col(3)); // eigenvalues/vectors are sorted in increasing order here ...

  // mean position and scale
  Matrix4d A_hat = A.transpose() * A;
  Matrix4d S_hat(Matrix4d::Zero());
  Vector4d b_hat = A.transpose() * b;

  Eigen::JacobiSVD<Matrix4d> svd(A_hat, Eigen::ComputeFullU | Eigen::ComputeFullV);
  for(int i=0; i<n; i++){
    if(svd.singularValues()[i]<eps)
      S_hat(i,i) = 0;
    else
      S_hat(i, i) = 1/svd.singularValues()[i];
  }
  if(cond)
    *cond = svd.singularValues()[0]/svd.singularValues()[n-1];

  Vector4d x = svd.matrixV()*S_hat*svd.matrixU().transpose()*b_hat;
  if(scale)
    *scale = x[3];

  pose.pose.position = eigenToGeometry_msgs(x.block<3,1>(0,0));
  pose.pose.orientation = eigenToGeometry_msgs(q_mean);

  return true;
}

}
} /* namespace msf_core */
