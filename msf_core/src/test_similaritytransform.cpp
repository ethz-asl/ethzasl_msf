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

using namespace msf_core;

int main(int argc, char** argv)
{

  similarity_transform::From6DoF T;
  const int N = 100;
  const double s_p = 0.1;
  const double s_q = 1e-2;

  // the pose we want to estimate
  Eigen::Vector3d p(Eigen::Vector3d::Random());
  Eigen::Quaterniond q(Eigen::Matrix<double, 4, 1>::Random());
  q.normalize();
  double scale = 2;

  std::cout << "Real pose: \n\tp: " << p.transpose() << "\n\tq(x y z w): " << q.coeffs().transpose() << std::endl;

  // generate a set of measurements
  for (int i = 0; i < N; i++)
  {
    Eigen::Vector3d p1;
    p1 = p + Eigen::Vector3d::Random() * 0.1 + Eigen::Vector3d(3, 4, 5);
    Eigen::Quaterniond q1(Eigen::Matrix<double, 4, 1>::Random());
    q1.normalize();

    Eigen::Vector3d p2 = (q1 * p + p1) / scale;
    Eigen::Quaterniond q2 = q1 * q;
    p2 += (Eigen::Vector3d::Random() * s_p);
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

    similarity_transform::Pose Pd;
    double cond;
    double _scale;
    T.compute(Pd, &_scale, &cond);

    Eigen::Vector3d pr = geometry_msgsToEigen(Pd.pose.position);
    Eigen::Quaterniond qr = geometry_msgsToEigen(Pd.pose.orientation);

    std::cout << "Result:\tp:" << pr.transpose() << "\tq: " << qr.coeffs().transpose() << "\tscale: " << _scale
        << std::endl;
    double p_err = (pr - p).norm();
//  Eigen::Quaterniond q_err = qr.inverse()*q;
    std::cout << "error\tp: " << p_err << "\tq: " << q.angularDistance(qr) * 180 / M_PI << "\tscale: "
        << std::abs(1.0 - scale / _scale) * 100 << "%\tcond: " << cond << std::endl;
  }

  return 0;
}
