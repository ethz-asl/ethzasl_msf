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
#include <msf_core/testing_entrypoint.h>
#include <msf_core/testing_predicates.h>

TEST(MSF_Core, XiMatrix) {
  using namespace msf_core;
  Eigen::Quaterniond q(Eigen::Matrix<double, 4, 1>::Random());
  q.normalize();

  // Xi matrix test.
  EXPECT_NEAR_EIGEN(XiMat(q.coeffs()).transpose() * q.coeffs(),
                    (Eigen::Matrix<double, 3, 1>::Zero()),
                    1e-5);
}

TEST(MSF_Core, GeometryMsgsCovBlockToEigen) {
  using namespace msf_core;
  // Block writing test.
  similarity_transform::Pose::_covariance_type cov;
  for (int r = 0; r < 6; r++)
    for (int c = 0; c < 6; c++)
      cov[r + c * 6] = r * c;
  Eigen::Map<Matrix6> covm(cov.data());

  EXPECT_NEAR_EIGEN(GeometryMsgsCovBlockToEigen(cov, geometry_msgs::cov::p,
                                                geometry_msgs::cov::p),
                    (covm.block<3, 3>(0, 0)), 1e-5);

  EXPECT_NEAR_EIGEN(GeometryMsgsCovBlockToEigen(cov, geometry_msgs::cov::q,
                                                geometry_msgs::cov::q),
                    (covm.block<3, 3>(3, 3)), 1e-5);

  EXPECT_NEAR_EIGEN(GeometryMsgsCovBlockToEigen(cov, geometry_msgs::cov::p,
                                                geometry_msgs::cov::q),
                    (covm.block<3, 3>(0, 3)), 1e-5);

  EXPECT_NEAR_EIGEN(GeometryMsgsCovBlockToEigen(cov, geometry_msgs::cov::q,
                                                geometry_msgs::cov::p),
                    (covm.block<3, 3>(3, 0)), 1e-5);
}

TEST(MSF_Core, EigenCovBlockToGeometryMsgs) {
  using namespace msf_core;
  Matrix3 covp;
  covp << 31, 32, 33, 32, 34, 35, 33, 35, 36;
  Matrix3 covq;
  covq << 21, 22, 23, 22, 24, 25, 23, 25, 26;
  Matrix3 covpq;
  covpq << 1, 2, 3, 4, 5, 6, 7, 8, 9;

  similarity_transform::Pose::_covariance_type cov;
  for (int r = 0; r < 6; r++)
    for (int c = 0; c < 6; c++)
      cov[r + c * 6] = r * c;
  Eigen::Map<Matrix6> covm(cov.data());

  EigenCovBlockToGeometryMsgs(cov, covq, geometry_msgs::cov::q,
                              geometry_msgs::cov::q);
  EigenCovBlockToGeometryMsgs(cov, covp, geometry_msgs::cov::p,
                              geometry_msgs::cov::p);

  EigenCovBlockToGeometryMsgs(cov, covpq, geometry_msgs::cov::p,
                              geometry_msgs::cov::q);

  EigenCovBlockToGeometryMsgs(cov, covpq.transpose(), geometry_msgs::cov::q,
                              geometry_msgs::cov::p);

  EXPECT_NEAR_EIGEN((covm.block<3, 3>(0, 0)), covp, 1e-5);

  EXPECT_NEAR_EIGEN((covm.block<3, 3>(3, 3)), covq, 1e-5);

  EXPECT_NEAR_EIGEN((covm.block<3, 3>(0, 3)), covpq, 1e-5);

  EXPECT_NEAR_EIGEN((covm.block<3, 3>(3, 0)), covpq.transpose(), 1e-5);
}

TEST(MSF_Core, SimilarityTransform) {
  using namespace msf_core;

  similarity_transform::From6DoF T;
  const int N = 100;
  const double s_p = 1e-2;
  const double s_q = 1e-2;

  // The pose we want to estimate.
  Vector3 p(Vector3::Random());
  Eigen::Quaterniond q(Eigen::Matrix<double, 4, 1>::Random());
  q.normalize();
  double scale = 2;

  // Generate a set of measurements.
  for (int i = 0; i < N; i++) {
    Vector3 p1;
    p1 = p + Vector3::Random() * 0.1 + Vector3(3, 4, 5);
    Eigen::Quaterniond q1(Eigen::Matrix<double, 4, 1>::Random());
    q1.normalize();

    Vector3 p2 = (q1 * p + p1) / scale;
    Eigen::Quaterniond q2 = q1 * q;
    p2 += (Vector3::Random() * s_p);
    Eigen::Quaterniond q_noise(Eigen::Quaterniond::Identity());
    q_noise.coeffs() += Eigen::Matrix<double, 4, 1>::Random() * s_q;
    q_noise.normalize();
    q2 = q2 * q_noise;

    similarity_transform::Pose P1;
    similarity_transform::Pose P2;
    P1.pose.position = EigenToGeometryMsgs(p1);
    P1.pose.orientation = EigenToGeometryMsgs(q1);
    P2.pose.position = EigenToGeometryMsgs(p2);
    P2.pose.orientation = EigenToGeometryMsgs(q2);

    T.AddMeasurement(P1, P2);
  }

  // Estimate pose.
  similarity_transform::Pose Pd;
  double cond;
  double _scale;
  T.Compute(Pd, &_scale, &cond);

  Vector3 pr = GeometryMsgsToEigen(Pd.pose.position);
  Eigen::Quaterniond qr = GeometryMsgsToEigen(Pd.pose.orientation);

  EXPECT_NEAR_EIGEN(pr, p, s_p);
  EXPECT_NEAR_EIGEN(qr.coeffs(), q.coeffs(), s_q);
}

MSF_UNITTEST_ENTRYPOINT
