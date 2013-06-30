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

#ifndef SEGMENT_PLANNING_H_
#define SEGMENT_PLANNING_H_

#include <mav_planning_utils/motion4D.h>
#include <mav_planning_utils/polynomial.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace mav_planning_utils
{

namespace segment_planning
{

/**
 * \brief Quadratic optimization for a single polynomial
 */
template<int N, typename Derived>
  Polynomial<N, typename Derived::Scalar> optimizePolynomial1D(
      double dt,
      const Eigen::MatrixBase<Derived> & constraints,
      const Eigen::MatrixBase<Derived> & constraints_time,
      const Eigen::Matrix<int, Derived::RowsAtCompileTime, 1> & constraints_type,
      int derivative_to_optimize)
  {
    typedef typename Derived::Scalar Scalar;
    typedef Polynomial<N, Scalar> P;
    const int n_constraints = Derived::RowsAtCompileTime;
    const int n_all = n_constraints + N;

    EIGEN_STATIC_ASSERT(n_constraints<=N, YOU_MADE_A_PROGRAMMING_MISTAKE);

    const double tol = 1e-12;

    Eigen::Matrix<Scalar, n_all, n_all> Q;
    Eigen::Matrix<Scalar, n_all, 1> b, x;

    Q.setZero();
    b.setZero();

    P::quadraticCostJacobian(Q.template block<N, N>(0, 0), dt, derivative_to_optimize);

    /*
     * Q =
     * [ H      A_eq'       ]
     * [ A_eq   zeros(n_eq) ]
     */
    for (int i = 0; i < n_constraints; i++)
    {
      const double & constraint_time = constraints_time[i];
      const int & constraint_type = constraints_type[i];

      P::baseCoeffsWithTime(Q.template block<1, N>(N + i, 0), constraint_type, constraint_time);
      b[N + i] = constraints[i];
    }

    Q.template block<N, n_constraints>(0, N) = Q.template block<n_constraints, N>(N, 0).transpose();

    x = pseudoInverseSolver(Q, b, tol);

    return P(x.template head<N>());
  }



template<int n_ip, int n_iy, int n_fp = n_ip, int n_fy = n_iy>
  class Options4D
  {
  public:
    Eigen::Matrix<int, n_ip, 1> initial_constraints_pos_fixed;
    Eigen::Matrix<int, n_iy, 1> initial_constraints_yaw_fixed;
    Eigen::Matrix<int, n_fp, 1> final_constraints_pos_fixed;
    Eigen::Matrix<int, n_fy, 1> final_constraints_yaw_fixed;
    int degree_to_optimize_pos;
    int degree_to_optimize_yaw;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Options4D()
    {
      initial_constraints_pos_fixed.setConstant(DerivativesP::s);
      initial_constraints_yaw_fixed.setConstant(DerivativesO::wd);
      degree_to_optimize_pos = DerivativesP::s;
      degree_to_optimize_yaw = DerivativesO::wd;
    }
  };

template<int n_pos = 13, int n_yaw = 8>
  class Polynomial4dOptimizer
  {
  public:
    enum
    {
      N_POS = n_pos, N_YAW = n_yaw
    };

    typedef Polynomial<n_pos> MdlPos;
    typedef Polynomial<n_yaw> MdlYaw;

  private:
    Polynomial<n_pos> mdl_x_;
    Polynomial<n_pos> mdl_y_;
    Polynomial<n_pos> mdl_z_;
    Polynomial<n_yaw> mdl_yaw_;
    double segment_time_;

    template<int n_i, int n_f, int n_a>
      void assembleConstraintVectors(const Eigen::Matrix<double, n_a, 1> & initial_states,
                                     const Eigen::Matrix<int, n_i, 1> & initial_states_fixed,
                                     const Eigen::Matrix<double, n_a, 1> & final_states, double final_states_time,
                                     const Eigen::Matrix<int, n_f, 1> & final_states_fixed, int degree_to_optimize,
                                     Eigen::Matrix<double, n_i + n_f, 1> & constraints,
                                     Eigen::Matrix<double, n_i + n_f, 1> & constraints_time,
                                     Eigen::Matrix<int, n_i + n_f, 1> & constraints_type)
      {
        int dof;
        for (int i = 0; i < n_i; i++)
        {
          dof = initial_states_fixed[i];
          constraints[i] = initial_states[dof];
          constraints_time[i] = 0;
          constraints_type[i] = dof;
        }

        for (int i = 0; i < n_f; i++)
        {
          dof = final_states_fixed[i];
          constraints[n_i + i] = final_states[dof];
          constraints_time[n_i + i] = final_states_time;
          constraints_type[n_i + i] = dof;
        }
      }

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//  MavPolyMotion(){};

    template<int n_ip, int n_iy, int n_fp, int n_fy, int n_p, int n_y>
      bool optimizePath(double segment_time, const Motion4D<n_p, n_y> & initial_states,
                        const Motion4D<n_p, n_y> & final_states,
                        const Options4D<n_ip, n_iy, n_fp, n_fy> & options)
      {
        assert(segment_time >= 0);
        segment_time_ = segment_time;
        const int n_cp = n_ip + n_fp;
        const int n_cy = n_iy + n_fy;
        Motion4D<n_cp, n_cy> constraints; // this is not longer ordered by the number of derivatives!
        Eigen::Matrix<double, n_cp, 1> constraints_time_p;
        Eigen::Matrix<double, n_cy, 1> constraints_time_y;
        Eigen::Matrix<int, n_cp, 1> constraints_type_p;
        Eigen::Matrix<int, n_cy, 1> constraints_type_y;

        assembleConstraintVectors(initial_states.x, options.initial_constraints_pos_fixed, final_states.x, segment_time,
                                  options.final_constraints_pos_fixed, options.degree_to_optimize_pos, constraints.x,
                                  constraints_time_p, constraints_type_p);
        assembleConstraintVectors(initial_states.y, options.initial_constraints_pos_fixed, final_states.y, segment_time,
                                  options.final_constraints_pos_fixed, options.degree_to_optimize_pos, constraints.y,
                                  constraints_time_p, constraints_type_p);
        assembleConstraintVectors(initial_states.z, options.initial_constraints_pos_fixed, final_states.z, segment_time,
                                  options.final_constraints_pos_fixed, options.degree_to_optimize_pos, constraints.z,
                                  constraints_time_p, constraints_type_p);
        assembleConstraintVectors(initial_states.yaw, options.initial_constraints_yaw_fixed, final_states.yaw,
                                  segment_time, options.final_constraints_yaw_fixed, options.degree_to_optimize_yaw,
                                  constraints.yaw, constraints_time_y, constraints_type_y);

        mdl_x_ = optimizePolynomial1D<n_pos>(segment_time, constraints.x, constraints_time_p, constraints_type_p,
                                 options.degree_to_optimize_pos);
        mdl_y_ = optimizePolynomial1D<n_pos>(segment_time, constraints.y, constraints_time_p, constraints_type_p,
                                 options.degree_to_optimize_pos);
        mdl_z_ = optimizePolynomial1D<n_pos>(segment_time, constraints.z, constraints_time_p, constraints_type_p,
                                 options.degree_to_optimize_pos);
        mdl_yaw_ = optimizePolynomial1D<n_yaw>(segment_time, constraints.yaw, constraints_time_y, constraints_type_y,
                                   options.degree_to_optimize_yaw);

        return true;
      }

    template<int n_ip, int n_iy, int n_fp, int n_fy, int n_p, int n_y>
      bool optimizePathAndTime(double segment_time, const Motion4D<n_p, n_y> & initial_states,
                               const Motion4D<n_p, n_y> & final_states,
                               const Options4D<n_ip, n_iy, n_fp, n_fy> & options,
                               const Eigen::Matrix<double, n_p, 1> & max_pos,
                               const Eigen::Matrix<double, n_y, 1> & max_yaw, double dt = -1, double * path_time = NULL)
      {
        const int n_samples = n_pos * 2;
        const double dt_samples = segment_time / static_cast<double>(n_samples);
        segment_time_ = segment_time;

        typedef mav_planning_utils::Motion4D<n_p, n_y> PathEl;
        typename PathEl::Vector path(n_samples);

        this->template optimizePath(segment_time, initial_states, final_states, options);
        this->template samplePath<n_p, n_y>(segment_time, dt_samples, path);

        double val;
        double time_scale = 0;

        // start from 1 since we don't really have a constraint in position which would require the time to be scaled
        for (int i = 1; i < n_p; i++)
        {
          for (int j = 0; j < n_samples; j++)
          {
            const PathEl & p = path[j];
            val = sqrt(p.x[i] * p.x[i] + p.y[i] * p.y[i] + p.z[i] * p.z[i]) / max_pos[i];
            val = pow(val, 1.0 / static_cast<double>(i));
            if (val > time_scale)
              time_scale = val;
          }
        }

        // start from 1 since we don't really have a constraint in yaw which would require the time to be scaled
        for (int i = 1; i < n_y; i++)
        {
          for (int j = 0; j < n_samples; j++)
          {
            val = path[j].yaw[i] / max_yaw[i];
            val = pow(val, 1.0 / static_cast<double>(i));
            if (val > time_scale)
              time_scale = val;
          }
        }

        double new_time = time_scale * segment_time;

        if (dt != -1)
          new_time = ceil(new_time / dt) * dt;

        this->template optimizePath(new_time, initial_states, final_states, options);

        segment_time_ = new_time;

        if (path_time)
          *path_time = new_time;

        return true;
      }

    template<int n_p, int n_y>
      void samplePath(double segment_time, double dt, typename Motion4D<n_p, n_y>::Vector & result) const
      {
        int n_samples = (segment_time + dt * 0.5) / dt + 1;
        // hack around numerical badness ^^^^

        result.resize(n_samples);

        double t = 0;
        for (int i = 0; i < n_samples; i++)
        {
          Motion4D<n_p, n_y> &sample = result[i];
          mdl_x_.evaluate(sample.x, t);
          mdl_y_.evaluate(sample.y, t);
          mdl_z_.evaluate(sample.z, t);
          mdl_yaw_.evaluate(sample.yaw, t);
          t += dt;
        }
      }

    template<int n_p, int n_y>
      void samplePath(double time, Motion4D<n_p, n_y> & result) const
      {
        mdl_x_.evaluate(result.x, time);
        mdl_y_.evaluate(result.y, time);
        mdl_z_.evaluate(result.z, time);
        mdl_yaw_.evaluate(result.yaw, time);
      }

    Polynomial<n_pos> getMdlX() const
    {
      return mdl_x_;
    }

    Polynomial<n_pos> getMdlY() const
    {
      return mdl_y_;
    }

    Polynomial<n_pos> getMdlZ() const
    {
      return mdl_z_;
    }

    Polynomial<n_yaw> getMdlYaw() const
    {
      return mdl_yaw_;
    }

    double segmentTime() const
    {
      return segment_time_;
    }
  };

} // segment_planning

} // mav_planning_utils

#endif /* HELI_MODEL_H_ */
