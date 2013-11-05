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

#ifndef CONVERSIONS_H_
#define CONVERSIONS_H_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <mav_planning_utils/mav_state.h>
#include <mav_planning_utils/motion4D.h>
#include <mav_planning_utils/motion_defines.h>
#include <mav_planning_utils/sincos.h>


namespace mav_planning_utils{

inline void applyMulticopterModelOld(const Eigen::Vector3d & acceleration, const Eigen::Vector3d & jerk, double yaw, double yaw_dot,
                    Eigen::Quaterniond & q, Eigen::Vector3d & acceleration_b, Eigen::Vector3d & angvel_b)
{
  //  zb = acc+[0 0 9.81]';
  //  thrust =  norm(zb);
  //  zb = zb / thrust;
  //
  //  xc = [cos(yaw) sin(yaw) 0]';
  //
  //  yb = cross(zb, xc);
  //  yb = yb/norm(yb);
  //
  //  xb = cross(yb, zb);
  //
  //  q(:,i) = rot2quat([xb yb zb]);
  //
  //  h_w = 1/thrust*(acc_dot - (zb'*acc_dot)*zb);
  //
  //  w(1,i) = -h_w'*yb;
  //  w(2,i) = h_w'*xb;
  //  w(3,i) = yaw_dot*[0 0 1]*zb;

  Eigen::Vector3d xb;
  Eigen::Vector3d yb;
  Eigen::Vector3d zb(acceleration);

  zb[2] += 9.81;
  double thrust = zb.norm();
  zb = zb / thrust;

  double s_yaw, c_yaw;
  sincos(yaw, &s_yaw, &c_yaw);
  yb = zb.cross(Eigen::Vector3d(c_yaw, s_yaw, 0));
  yb.normalize();

  xb = yb.cross(zb);

  Eigen::Matrix3d R;
  R << xb, yb, zb;

  Eigen::Vector3d h_w = 1.0 / thrust * (jerk - (zb*(zb.transpose() * jerk)));

  q = Eigen::Quaterniond(R);
  acceleration_b = R.transpose() * zb * thrust;
  angvel_b[0] = -h_w.transpose() * yb;
  angvel_b[1] = h_w.transpose() * xb;
  angvel_b[2] = yaw_dot * zb[2];
}

inline void applyMulticopterModel(const Eigen::Vector3d & acceleration, const Eigen::Vector3d & jerk, double yaw, double yaw_dot,
                    Eigen::Quaterniond & q, Eigen::Vector3d & acceleration_b, Eigen::Vector3d & angvel_b)
{
  //  zb = acc+[0 0 9.81]';
  //  thrust =  norm(zb);
  //  zb = zb / thrust;
  //
  //  xc = [cos(yaw) sin(yaw) 0]';
  //
  //  yb = cross(zb, xc);
  //  yb = yb/norm(yb);
  //
  //  xb = cross(yb, zb);
  //
  //  q(:,i) = rot2quat([xb yb zb]);
  //
  //  h_w = 1/thrust*acc_dot);
  //
  //  w(1,i) = -h_w'*yb;
  //  w(2,i) = h_w'*xb;
  //  w(3,i) = yaw_dot*[0 0 1]*zb;

  Eigen::Vector3d xb;
  Eigen::Vector3d yb;
  Eigen::Vector3d zb(acceleration);

  zb[2] += 9.81;
  const double thrust = zb.norm();
  const double inv_thrust = 1.0 / thrust;
  zb = zb * inv_thrust;

  double s_yaw, c_yaw;
  sincos(yaw, &s_yaw, &c_yaw);
  yb = zb.cross(Eigen::Vector3d(c_yaw, s_yaw, 0));
  yb.normalize();

  xb = yb.cross(zb);

  const Eigen::Matrix3d R((Eigen::Matrix3d() << xb, yb, zb).finished());

  const Eigen::Vector3d h_w = inv_thrust * jerk;

  q = Eigen::Quaterniond(R);
  acceleration_b = R.transpose() * zb * thrust;
  angvel_b[0] = -h_w.transpose() * yb;
  angvel_b[1] = h_w.transpose() * xb;
  angvel_b[2] = yaw_dot * zb[2];
}

template<int n_pos, int n_yaw, class T>
  inline void motion4dToMulticopter(MavState & mc_state, const Motion4D<n_pos, n_yaw, T> & m4d)
  {
    EIGEN_STATIC_ASSERT(n_pos==5, THIS_METHOD_IS_ONLY_FOR_VECTORS_OF_A_SPECIFIC_SIZE)
    EIGEN_STATIC_ASSERT(n_yaw>=2, THIS_METHOD_IS_ONLY_FOR_VECTORS_OF_A_SPECIFIC_SIZE)

    // compute motion dependent states
    applyMulticopterModel(m4d.getStateP(DerivativesP::a), m4d.getStateP(DerivativesP::j), m4d.yaw[DerivativesO::o],
                          m4d.yaw[DerivativesO::w], mc_state.q, mc_state.a_b, mc_state.w_b);

    // copy remaining states
    mc_state.p = m4d.getStateP(DerivativesP::p);
    mc_state.v = m4d.getStateP(DerivativesP::v);
    mc_state.a = m4d.getStateP(DerivativesP::a);
    mc_state.j = m4d.getStateP(DerivativesP::j);
    mc_state.s = m4d.getStateP(DerivativesP::s);
  }

template<int n_pos, int n_yaw, class T>
  inline void multicopterToMotion4d(Motion4D<n_pos, n_yaw, T> & m4d, const MavState & mc_state)
  {
    EIGEN_STATIC_ASSERT(n_pos==5, THIS_METHOD_IS_ONLY_FOR_VECTORS_OF_A_SPECIFIC_SIZE)
    EIGEN_STATIC_ASSERT(n_yaw==3, THIS_METHOD_IS_ONLY_FOR_VECTORS_OF_A_SPECIFIC_SIZE)

    m4d.setStateP(DerivativesP::p, mc_state.p);
    m4d.setStateP(DerivativesP::v, mc_state.v);
    m4d.setStateP(DerivativesP::a, mc_state.a);
    m4d.setStateP(DerivativesP::j, mc_state.j);
    m4d.setStateP(DerivativesP::s, mc_state.s);
  }

template<class T>
  T yawFromQuaternion(const Eigen::Quaternion<T> & q)
  {
    return atan2(2.0 * (q.w() * q.z() + q.x() * q.y()), 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
  }

template<class T>
  Eigen::Quaternion<T> quaternionFromYaw(T yaw)
  {
    const T yaw_2 = yaw / 2.0;
    return Eigen::Quaternion<T>(cos(yaw_2), 0, 0, sin(yaw_2));
  }

template<class Derived>
  Eigen::Quaternion<typename Derived::Scalar> QuaternionFromSmallAngle(const Eigen::MatrixBase<Derived> & theta)
  {
  typedef typename Derived::Scalar Scalar;
  EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
  const Scalar q_squared = theta.squaredNorm() / 4.0;

    if ( q_squared < 1)
    {
      return Eigen::Quaternion<Scalar>(sqrt(1 - q_squared), theta[0] * 0.5, theta[1] * 0.5, theta[2] * 0.5);
    }
    else
    {
      const Scalar w = 1.0 / sqrt(1 + q_squared);
      const Scalar f = w*0.5;
      return Eigen::Quaternion<Scalar>(w, theta[0] * f, theta[1] * f, theta[2] * f);
    }
  }


}



#endif /* CONVERSIONS_H_ */
