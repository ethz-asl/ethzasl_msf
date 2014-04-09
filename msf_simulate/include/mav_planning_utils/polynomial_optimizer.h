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

#ifndef MINIMUM_SNAP_TRAJECTORY_H_
#define MINIMUM_SNAP_TRAJECTORY_H_

#include <Eigen/Eigen>
#include <Eigen/SVD>

#include <iostream>
#include <utility>
#include <vector>

#include <mav_planning_utils/motion_defines.h>

namespace mav_planning_utils
{

template<int N>
  class PolynomialOpt
  {
    typedef Eigen::Matrix<double, 1, N> VectorR;
    typedef Eigen::Matrix<double, N, 1> VectorV;
    typedef Eigen::Matrix<double, N, N> MatrixSq;
    typedef std::pair<VectorR, double> Constraint;
    typedef std::vector<Constraint, Eigen::aligned_allocator<Constraint> > ConstraintVector;

    enum ConstraintType
    {
      equality, inequality
    };

  private:
    MatrixSq H_;
    MatrixSq derivative_coefficients_;
    MatrixSq coefficients_;
    const static int DEG = N - 1;
    ConstraintVector equality_constraints_;
    ConstraintVector inequality_constraints_;

    void setupH(double t, int deg)
    {
      H_.setZero();

      for (int col = 0; col < N - deg; col++)
      {
        for (int row = 0; row < N - deg; row++)
        {
          double exp = (DEG - deg) * 2 + 1 - row - col;
          //          H_(row, col) = exp;
          H_(DEG-row, DEG-col) = derivative_coefficients_(deg, N - 1 - row) * derivative_coefficients_(deg, N - 1 - col) * pow(t, exp) * 2 / exp;
        }
      }
    }

    /// compute "coefficients in front of the coefficients"
    void computeDerivativeCoefficients()
    {

      derivative_coefficients_.setZero();
      derivative_coefficients_.row(0) = VectorR::Ones();

      int order = DEG;
      for (int n = 1; n < N; n++ )
      {
        for (int i = DEG - order; i < N; i++)
        {
          derivative_coefficients_(n, i) = (order - DEG + i) * derivative_coefficients_(n-1, i);
        }
        order --;
      }

//      std::cout << "coeffs: \n" << derivative_coefficients_ << std::endl;
    }

    void addConstraint(double t, double val, int deg, int constraint_type)
    {
      Constraint c(derivative_coefficients_.row(deg), val);

      double exp = 1;
      for (int i = deg + 1; i < N; i++)
      {
        c.first(0, i) *= pow(t, exp);
        //        c.first(0, i) = exp;
        exp += 1;
      }

      if (constraint_type == equality)
        equality_constraints_.push_back(c);
      else if (constraint_type == inequality)
        inequality_constraints_.push_back(c);
    }

    template<typename Derived>
      Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime, 1> solve(
          const Eigen::MatrixBase<Derived> & A,
          const Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime, 1> & b, double tol = 1e-12)
      {

        typedef Derived A_type;
        typedef Eigen::Matrix<typename A_type::Scalar, A_type::RowsAtCompileTime, 1> X_type;

//        Eigen::JacobiSVD<A_type> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::JacobiSVD<A_type> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);

        A_type U, V;
        X_type S;

        U = svd.matrixU();
        V = svd.matrixV();
        S = svd.singularValues();

        for (int i = 0; i < A.rows(); i++)
        {
          if (S[i] < tol)
            S[i] = 0;
          else
            S[i] = 1.0 / S[i];
        }

        return V * S.asDiagonal() * U.transpose() * b;
      }

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PolynomialOpt()
    {
      equality_constraints_.reserve(N);
      computeDerivativeCoefficients();
    }

    // doesn't work yet
//    void addInequalityConstraint(double t, double val, int deg)
//    {
//      addConstraint(t, val, deg, inequality);
//    }

    void resetEqualityConstraints()
    {
      equality_constraints_.clear();
      equality_constraints_.reserve(N);
    }

    void addEqualityConstraint(double t, double val, int deg)
    {
      addConstraint(t, val, deg, equality);
    }

    template<int n>
      void addEqualityConstraints(double t, const Eigen::Matrix<double, n, 1> & constraints)
      {
        assert(n <= N);
        for(int i=0; i<n; i++)
          addConstraint(t, constraints[i], i, equality);
      }

    void optimize(double t_0, double t_T, int dof_to_optimize)
    {
      assert(t_0==0);
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Q, V, U;
      Eigen::Matrix<double, Eigen::Dynamic, 1> b, x, S;

      int n_eq = equality_constraints_.size();
      int n_all = N + n_eq;
      const double tol = 1e-12;

      Q.resize(n_all, n_all);
      Q.setZero();
      b.resize(n_all);
      b.setZero();

      setupH(t_T, dof_to_optimize);
      Q.template block<N, N>(0, 0) = H_;

      /*
       * Q =
       * [ H      A_eq'       ]
       * [ A_eq   zeros(n_eq) ]
       */
      for (int i = 0; i < n_eq; i++)
      {
        Q.template block<1, N>(N + i, 0) = equality_constraints_[i].first;
        Q.template block<N, 1>(0, N + i) = equality_constraints_[i].first.transpose();
        b[N + i] = equality_constraints_[i].second;
      }

      x = solve(Q, b, tol);

      std::cout << "\ncoeffs: \n" << x.template head<N>().transpose()<<std::endl;

      for (int i = 0; i < N; i++)
        coefficients_.row(i) = derivative_coefficients_.row(i).cwiseProduct(x.template head<N>().transpose());

        //      std::cout<<"\ncoeffs: \n"<<coefficients_;

      }

      template <typename Derived>
      void optimize(double t_0, double t_T, const Eigen::MatrixBase<Derived> & constraints, const Eigen::MatrixBase<Derived> & constraints_time, const Eigen::Matrix<int, Derived::RowsAtCompileTime, 1> & constraints_type, int dof_to_optize)
    {
      typedef typename Derived::Scalar Scalar;
      const int n_constraints = Derived::RowsAtCompileTime;
      const int n_all = n_constraints + N;

      const double tol = 1e-12;

      Eigen::Matrix<Scalar, n_all, n_all> Q;
      Eigen::Matrix<Scalar, n_all, 1> b, x;

      Q.setZero();
      b.setZero();

      setupH(t_T, dof_to_optize);
      Q.template block<N, N>(0, 0) = H_;



      /*
       * Q =
       * [ H      A_eq'       ]
       * [ A_eq   zeros(n_eq) ]
       */
      for (int i = 0; i < n_constraints; i++)
      {
        const double & constraint_time = constraints_time[i];
        const int & constraint_type = constraints_type[i];

        // first coefficient doesn't get multiplied
        Q(N + i, constraint_type) = derivative_coefficients_(constraint_type, constraint_type);
        b[N + i] = constraints[i];

        if(constraint_time==0)
          continue;

        double t = constraint_time;
        for (int j = constraint_type + 1; j < N; j++)
        {
          Q(N + i, j) = derivative_coefficients_(constraint_type, j) * t;
          t = t*constraint_time;
        }
      }

      Q.template block<N, n_constraints>(0, N) = Q.template block<n_constraints, N>(N, 0).transpose();

      x = solve(Q, b, tol);

//      std::cout << "\ncoeffs: \n" << x.template head<N>().transpose()<<std::endl;
//      std::cout << "\nQ: \n" << Q<<std::endl;

      for (int i = 0; i < N; i++)
        coefficients_.row(i) = derivative_coefficients_.row(i).cwiseProduct(x.template head<N>().transpose());
    }

    /// evaluates the polynomial at time t and writes the result to result
    template <int max_deg> void evaluatePolynomials(double t, Eigen::Matrix<double, max_deg, 1> & result) const
        {
          assert(result.rows() <= DEG);

          VectorR tv;
          tv[0] = 1;

          for(int i=1; i<N; i++)
          tv[i] = t * tv[i-1];

//          std::cout<<"\ntvec:\n"<<tv<<std::endl;

          for(int i=0; i<max_deg; i++)
          {
            result[i] = 0;
            for(int j=0; j<N-i; j++)
            {
              result[i] += (coefficients_(i, j+i) * tv[j]);
            }
          }
        }

    /// evaluates the polynomial at time t and returns the result
    template <int max_deg> inline Eigen::Matrix<double, max_deg, 1> evaluatePolynomials(double t) const
        {
          Eigen::Matrix<double, max_deg, 1> result;
          evaluatePolynomials(t, result);
          return result;
        }

    /// evaluates the polynomial at times in t and writes the result for each time into the corresponding column of result
    template <int max_deg, int n_samples> void evaluatePolynomials(const Eigen::Matrix<double, 1, n_samples> & t, Eigen::Matrix<double, max_deg, n_samples> result) const
        {
          Eigen::Matrix<double, max_deg, 1> _result;
          for(int i=0; i<n_samples; i++)
          {
            evaluatePloynomials(t[i], _result);
            result.col(i) = _result;
          }
        }


  };


template<int N>
  void addEqualityConstraints(PolynomialOpt<N> & ms, double t, double p, double v, double a, double j, double s)
  {
    ms.addEqualityConstraint(t, p, DerivativesP::p);
    ms.addEqualityConstraint(t, v, DerivativesP::v);
    ms.addEqualityConstraint(t, a, DerivativesP::a);
    ms.addEqualityConstraint(t, j, DerivativesP::j);
    ms.addEqualityConstraint(t, s, DerivativesP::s);
  }

template<int N>
  void addEqualityConstraints(PolynomialOpt<N> & ms, double t, double p, double v, double a, double j)
  {
    ms.addEqualityConstraint(t, p, DerivativesP::p);
    ms.addEqualityConstraint(t, v, DerivativesP::v);
    ms.addEqualityConstraint(t, a, DerivativesP::a);
    ms.addEqualityConstraint(t, j, DerivativesP::j);
  }

template<int N>
  void addEqualityConstraints(PolynomialOpt<N> & ms, double t, double p, double v, double a)
  {
    ms.addEqualityConstraint(t, p, DerivativesP::p);
    ms.addEqualityConstraint(t, v, DerivativesP::v);
    ms.addEqualityConstraint(t, a, DerivativesP::a);
  }

} // end namespace
#endif /* MINIMUM_SNAP_TRAJECTORY_H_ */
