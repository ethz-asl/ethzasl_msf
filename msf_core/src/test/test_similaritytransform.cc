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

using namespace msf_core;

int main(int argc, char** argv)
{

  similarity_transform::From6DoF T;
  const int N = 100;
  const double s_p = 0.1;
  const double s_q = 1e-2;

  // the pose we want to estimate
  Vector3 p(Vector3::Random());
  Eigen::Quaterniond q(Eigen::Matrix<double, 4, 1>::Random());
  q.normalize();
  double scale = 2;

  std::cout << "Real pose: \n\tp: " << p.transpose() << "\n\tq(x y z w): " << q.coeffs().transpose() << std::endl;

  // generate a set of measurements
  for (int i = 0; i < N; i++)
  {
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
    //std::cout<<"q_n: "<<q_noise.coeffs().transpose()<<" "<<q_noise.norm()<<std::endl;
    q2 = q2 * q_noise;

    similarity_transform::Pose P1;
    similarity_transform::Pose P2;
    P1.pose.position = eigenToGeometry_msgs(p1);
    P1.pose.orientation = eigenToGeometry_msgs(q1);
    P2.pose.position = eigenToGeometry_msgs(p2);
    P2.pose.orientation = eigenToGeometry_msgs(q2);

    //std::cout<<"pose: \n\tp: "<<p1.transpose()<<"\tq: "<<q1.coeffs().transpose()<<std::endl;

    T.addMeasurement(P1, P2);
  }

  // estimate pose
  similarity_transform::Pose Pd;
  double cond;
  double _scale;
  T.compute(Pd, &_scale, &cond);

  Vector3 pr = geometry_msgsToEigen(Pd.pose.position);
  Eigen::Quaterniond qr = geometry_msgsToEigen(Pd.pose.orientation);

  std::cout << "\n#####\nResult:\tp:" << pr.transpose() << "\tq: " << qr.coeffs().transpose() << "\tscale: " << _scale
      << std::endl;
  double p_err = (pr - p).norm();

  std::cout << "error\tp: " << p_err << "\tq: " << q.angularDistance(qr) * 180 / M_PI << "\tscale: "
      << std::abs(1.0 - scale / _scale) * 100 << "%\tcond: " << cond << std::endl;


  // Xi matrix test
  std::cout << "\n#####\ntest Xi: \n" << xiMat(q.coeffs()).transpose() * q.coeffs() << "\nshould be all 0" << std::endl;


  // block writing test
  similarity_transform::Pose::_covariance_type cov;
  for (int r = 0; r < 6; r++)
    for (int c = 0; c < 6; c++)
      cov[r + c * 6] = r * c;
  Eigen::Map<Matrix6 > covm(cov.data());

  std::cout<<"\n#####\ntest accessing blocks of cov: \n"<<covm<<
      "\np:\n"<<geometry_msgsCovBlockToEigen(cov, geometry_msgs::cov::p, geometry_msgs::cov::p)<<
      "\nq:\n"<<geometry_msgsCovBlockToEigen(cov, geometry_msgs::cov::q, geometry_msgs::cov::q)<<
      "\npq:\n"<<geometry_msgsCovBlockToEigen(cov, geometry_msgs::cov::p, geometry_msgs::cov::q)<<
      "\nqp:\n"<<geometry_msgsCovBlockToEigen(cov, geometry_msgs::cov::q, geometry_msgs::cov::p)<<
      std::endl;

  std::cout<<"\n#####\ntest writing blocks of cov: "<<std::endl;
  Matrix3 covp;
  covp << 31, 32, 33, 32, 34, 35, 33, 35, 36;
  Matrix3 covq;
  covq << 21, 22, 23, 22, 24, 25, 23, 25, 26;
  Eigen::Matrix<double, 4, 4> covpq;
  covpq << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16; // just for fun a bigger one to test blocks
  std::cout<<
      "\ncovp:\n"<<covp<<
      "\ncovq:\n"<<covq<<
      "\ncovpq (we will only take the block starting at 1,1):\n"<<covpq<<
      std::endl;
  eigenCovBlockToGeometry_msgs(cov, covp, geometry_msgs::cov::p, geometry_msgs::cov::p);
  eigenCovBlockToGeometry_msgs(cov, covq, geometry_msgs::cov::q, geometry_msgs::cov::q);
  eigenCovBlockToGeometry_msgs(cov, covpq.block<3, 3>(1, 1), geometry_msgs::cov::p, geometry_msgs::cov::q);
  std::cout << "result: \n" << covm << std::endl;

  return 0;
}
