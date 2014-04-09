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

#ifndef PATH_PLANNING_H_
#define PATH_PLANNING_H_

#include <mav_planning_utils/polynomial.h>
#include <mav_planning_utils/motion4D.h>
#include <vector>
#include <algorithm>
#include <map>

namespace mav_planning_utils
{

namespace path_planning
{

enum
{
  DEFAULT_N = 12
};

template<int _D = 1>
  class Vertex
  {

  public:
    const static int D = _D;
    typedef Eigen::Matrix<double, D, 1> ConstraintValueT;
    typedef std::pair<int, ConstraintValueT> Constraint;
    typedef std::map<int, ConstraintValueT> Constraints;
    double time_to_next;
    int derivative_to_optimize;

  private:
      Constraints constraints;

  public:

    Vertex() :
        time_to_next(0), derivative_to_optimize(0)
    {
    }

    Vertex(double _time_to_next, int _derivative_to_optimze) :
        time_to_next(_time_to_next), derivative_to_optimize(_derivative_to_optimze)
    {
    }

    Vertex(double _time_to_next, int _derivative_to_optimze, int derivatives) :
        time_to_next(_time_to_next), derivative_to_optimize(_derivative_to_optimze)
    {
      setAllZero(derivatives);
    }

    void addConstraint(int type, double value)
    {
      constraints[type] = ConstraintValueT::Constant(value);
    }

    template<class Derived>
      void addConstraint(int type, const Eigen::MatrixBase<Derived> & c)
      {
        constraints[type] = c;
      }

    void setAllZero(int up_to_type)
    {
      for (int i = 0; i <= up_to_type; ++i)
      {
        constraints[i] = ConstraintValueT::Zero();
      }
    }

    bool hasConstraint(int type) const
    {
      typename Constraints::const_iterator it = constraints.find(type);
      return it != constraints.end();
    }

    ConstraintValueT getConstraint(int type) const
    {
      typename Constraints::const_iterator it = constraints.find(type);
      if (it != constraints.end())
        return it->second;
      else
        return ConstraintValueT::Zero(); // TODO : throw exception ...
    }

    typename Constraints::const_iterator cBegin() const
    {
      return constraints.begin();
    }

    typename Constraints::const_iterator cEnd() const
    {
      return constraints.end();
    }
  };


typedef Vertex<1> Vertex1D;
typedef Vertex<3> Vertex3D;
typedef Vertex<4> Vertex4D;

template<int _N = DEFAULT_N>
  class Segment
  {
  public:
    enum
    {
      N = _N
    };

    typedef Polynomial<N> Type;
    typedef std::vector<Segment<N> > Vector;

    Type p;
    double t;
    Segment() :
        t(0)
    {
    }

    Segment(const Type & _p, double _t) :
        p(_p), t(_t)
    {
    }
  };


template<int N = DEFAULT_N>
  typename Segment<N>::Vector optimizePolynomials1D(const std::vector<Vertex1D > & vertices, int continuity, double tol =
                                                        1e12)
  {
    typedef Polynomial<N> P;
    typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Q_t;
    typedef Eigen::Matrix<double, Eigen::Dynamic, 1> x_t;

    typedef Vertex1D::Constraints::const_iterator CIt;

    const int n_vertices = vertices.size();

    typename Segment<N>::Vector result;

    const int n_all = (n_vertices - 1) * N * 2;

    Q_t Q(n_all, n_all);
    x_t b(n_all), x(n_all);

    Q.setZero();
    b.setZero();

    int start_pos = N * (n_vertices - 1);
    for (int i = 0; i < n_vertices - 1; ++i)
    {
      int n_c = 0;
      const Vertex1D & cv = vertices[i];

      // fill in cost jacobian
      // equality constraints
      P::quadraticCostJacobian(Q.block<N, N>(i * N, i * N), cv.time_to_next, cv.derivative_to_optimize);

      // current vertex
      for (CIt it = cv.cBegin(); it != cv.cEnd(); ++it)
      {
        if (n_c > n_all/2)
        {
          std::cout<<"too many constraints: "<<n_c<<", but only "<<n_all/2<<" available\n";
          return result;
        }

        Q.block(start_pos + n_c, N * i, 1, N) = P::baseCoeffsWithTime(it->first, 0);
        b[start_pos + n_c] = it->second[0];
        ++n_c;
      }

//      std::cout << n_c << "constraints begin\n";

      // next vertex
      if (i < n_vertices - 1)
      {
        const Vertex1D & nv = vertices[i + 1];
        for (CIt it = nv.cBegin(); it != nv.cEnd(); ++it)
        {
          if (n_c > n_all / 2)
          {
            std::cout << "too many constraints: " << n_c << ", but only " << n_all / 2 << " available\n";
            return result;
          }

          Q.block(start_pos + n_c, N * i, 1, N) = P::baseCoeffsWithTime(it->first, cv.time_to_next);
          b[start_pos + n_c] = it->second[0];
          ++n_c;
        }
//        std::cout << n_c << "constraints end\n";
      }
      // continuity constraints, only for segments that do not yield to the end vertex
      if (i < n_vertices - 2)
      {
        const Vertex1D & nv = vertices[i + 1];
        for (int d = 0; d <= continuity; ++d)
        {
          if (!nv.hasConstraint(d))
          {
            if (n_c > n_all / 2)
            {
              std::cout << "too many constraints: " << n_c << ", but only " << n_all / 2 << " available\n";
              return result;
            }

            Q.block(start_pos + n_c, N * i, 1, N) = P::baseCoeffsWithTime(d, cv.time_to_next);
            Q.block(start_pos + n_c, N * i + N, 1, N) = P::baseCoeffsWithTime(d, 0) * (-1.0);
            b[start_pos + n_c] = 0;
            ++n_c;
          }
        }
//        std::cout << n_c << "constraints cont\n";

        // mirror A on diagonal including continuity constraints
        Q.block(N * i, start_pos, 2 * N, n_c) = Q.block(start_pos, N * i, n_c, N * 2).transpose();

      }
      else
        // mirror A on diagonal
        Q.block(N * i, start_pos, N, n_c) = Q.block(start_pos, N * i, n_c, N).transpose();

      start_pos += n_c;
    }

    Q_t _Q(start_pos, start_pos);
    _Q = Q.block(0, 0, start_pos, start_pos);
    x_t _b = b.head(start_pos);
    x = pseudoInverseSolver(_Q, _b);

    result.resize(n_vertices - 1);
    for (int i = 0; i < n_vertices - 1; ++i)
    {
//      result.push_back(Segment<N>(P(x.template segment<N>(i * N)), vertices[i].time_to_next));
      result[i].p = P(x.template segment<N>(i * N));
      result[i].t = vertices[i].time_to_next;
    }
    return result;
  }

template<int N = DEFAULT_N, class Derived>
  bool samplePath(const Eigen::MatrixBase<Derived> & result, const std::vector<Segment<N> > & path, double t)
  {
    double t_a = 0;
    typename Segment<N>::Vector::const_iterator it;

    for (it = path.begin(); it != path.end(); ++it)
    {
      t_a += it->t;
      if (t_a - t >= -1e-12)
        break;
    }
    if (it == path.end())
    {
      std::cout << "t longer than available path time:" << t << " / " << t_a << std::endl;
      return false;
    }

    it->p.evaluate(result, t - (t_a - it->t));
    return true;
  }


template<int _N = DEFAULT_N>
  class Path4D
  {
  public:
    enum{
      N = _N
    };
    typedef Segment<N> SegmentType;
    typedef typename SegmentType::Vector SegmentVector;

  private:
    SegmentVector sx_;
    SegmentVector sy_;
    SegmentVector sz_;
    SegmentVector syaw_;
    std::vector<Vertex4D> vertices_;

    /// turns 4D vertices into a 1D vertices determined by DoF for better internal handling
    std::vector<Vertex1D> vertices4Dto1D(const std::vector<Vertex4D> & v4, unsigned int DoF)
    {
      assert(DoF < 4);
      std::vector<Vertex1D> v1;
      v1.resize(v4.size());
      for (size_t i = 0; i < v4.size(); ++i)
      {
        for (Vertex4D::Constraints::const_iterator it = v4[i].cBegin(); it != v4[i].cEnd(); ++it)
        {
          v1[i].addConstraint(it->first, it->second[DoF]);
        }
        v1[i].derivative_to_optimize = v4[i].derivative_to_optimize;
        v1[i].time_to_next = v4[i].time_to_next;
      }

      return v1;
    }

//    template<int N>
//      Eigen::Matrix<double, 4, 1> sampleSegment(const Segment<N, 4> & s, double t, int derivative)
//      {
//        // TODO: check max derivative
//        Eigen::Matrix<double, 4, 1> res;
//        for (int i = 0; i < 4; ++i)
//          res[i] = s.p[i].evaluate(t, derivative);
//
//        return res;
//      }

  public:

    /// standard constructor, doesn't initialize anything
    Path4D()
    {
    }

    /// initializes the internal segments with the arguments, so that sample() can be used
    Path4D(const SegmentVector & sx, const SegmentVector &sy, SegmentVector &sz, const SegmentVector &syaw) :
        sx_(sx), sy_(sy), sz_(sz), syaw_(syaw)
    {
      assert(sx_.size() == sy_.size() == sz_.size() == syaw_.size());
    }

    /**
     * \brief plans a path through the points given in vertices.
     * For obtaining path data \sa void sample(), getSegmentsX(), getVertices()
     * \param vertices Vertices to plan the path through
     * \param continuity Defines up to which derivative the path should be continuous. Use DerivativeP for that.
     * \param tol Tolerance for Pseudo Inverse
     */
    bool optimize(const std::vector<Vertex4D> & vertices, int continuity, double tol = 1e12)
    {
      std::vector<Vertex1D> v1;
      v1 = vertices4Dto1D(vertices, 0);
      sx_ = optimizePolynomials1D<N>(v1, continuity, tol);
      v1 = vertices4Dto1D(vertices, 1);
      sy_ = optimizePolynomials1D<N>(v1, continuity, tol);
      v1 = vertices4Dto1D(vertices, 2);
      sz_ = optimizePolynomials1D<N>(v1, continuity, tol);
      v1 = vertices4Dto1D(vertices, 3);
      syaw_ = optimizePolynomials1D<N>(v1, continuity, tol);

      return true;
    }

    /**
     * \brief computes a scaling factor by which every segment time has to be scaled in order to stay within the given bounds
     * \param max_p Vertex with "constraints" for the motion derivatives. At least a constraint for velocity is required.
     * \param max_yaw Vertex with "constraints" for the motion derivatives. At least a constraint for angular velocity is required.
     * \return vector with time scaling factors for each segment
     */
    std::vector<double> computeTimeScales(const Vertex1D & max_p, const Vertex1D & max_yaw)
    {
      std::vector<double> ts(sx_.size());
      for (size_t is = 0; is < sx_.size(); ++is)
      {
        double time_scale = 0;
        for (double t = 0; t < sx_[is].t; t += 0.1)
        {
          for (typename Vertex1D::Constraints::const_iterator it = max_p.cBegin(); it != max_p.cEnd(); ++it)
          {
            const double x = sx_[is].p.evaluate(t, it->first);
            const double y = sy_[is].p.evaluate(t, it->first);
            const double z = sz_[is].p.evaluate(t, it->first);

            double val = sqrt(x * x + y * y + z * z) / it->second[0];
            //            std::cout<<"val "<<sqrt(x * x + y * y + z * z)<<" timescale "<< time_scale<<std::endl;
            val = pow(val, 1.0 / static_cast<double>(it->first));
            if (val > time_scale)
              time_scale = val;
          }

          for (typename Vertex1D::Constraints::const_iterator it = max_yaw.cBegin(); it != max_yaw.cEnd(); ++it)
          {
            const double yaw = syaw_[is].p.evaluate(t, it->first);

            double val = std::abs(yaw / it->second[0]);
            val = pow(val, 1.0 / static_cast<double>(it->first));
            if (val > time_scale)
              time_scale = val;
          }
        }

        ts[is]= time_scale;
      }
      return ts;
    }

    /**
     * \brief plans a path through the points given in vertices and optimized the segment times according to the given limits in max_p and max_yaw
     * The time_to_next members in vertices are ignored. In a first step, rough guesses of the segment times are made
     * based on the maximum velocity and a first path is planned. Then, it gets checked for limit violations, segment
     * times are scaled accordingly and the path gets re-planned based on the new times.
     * For obtaining path data \sa sample(), getSegmentsX(), getVertices()
     * \param vertices Vertices to plan the path through
     * \param continuity Defines up to which derivative the path should be continuous. Use DerivativeP for that.
     * \param max_p Vertex with "constraints" for the motion derivatives. At least a constraint for velocity is required.
     * \param max_yaw Vertex with "constraints" for the motion derivatives. At least a constraint for angular velocity is required.
     * \param time_multiplier If not 0, the optimized segment times get ceiled to a multiple of time_multiplier. To ease sampling, it's useful to set this to the same value as dt in sample().
     * \param tol Tolerance for Pseudo Inverse
     */
    bool optimizeWithTime(const std::vector<Vertex4D> & vertices, int continuity, const Vertex1D & max_p,
                          const Vertex1D & max_yaw, double time_multiplier = 0.1, double tol = 1e12)
    {
      if (!(max_p.hasConstraint(DerivativesP::v) && max_yaw.hasConstraint(DerivativesO::w)))
      {
        std::cout << "need velocity constraints for this to work\n";
        return false;
      }

      const double v_max_p = max_p.getConstraint(DerivativesP::v)[0];
      const double v_max_yaw = max_yaw.getConstraint(DerivativeO::w)[0];

      if (!(v_max_p > 0) && (v_max_yaw > 0))
      {
        std::cout << "maximum velocities have to be > 0, got " << v_max_p << " " << v_max_yaw << std::endl;
      }

      vertices_ = vertices;

      // rough guess of time first:
      for (std::vector<Vertex4D>::iterator it = vertices_.begin(); it != (vertices_.end() - 1); ++it)
      {
        if (!it->hasConstraint(DerivativesP::p))
        {
          std::cout << "need position / yaw constraints for this to work\n";
          return false;
        }

        std::vector<Vertex4D>::const_iterator it_next = it + 1;
        const Vertex4D::ConstraintValueT cc = it->getConstraint(DerivativesP::p);
        const Vertex4D::ConstraintValueT nc = it_next->getConstraint(DerivativesP::p);

        const double tp = (cc.head<3>() - nc.head<3>()).norm() / v_max_p;
        const double ty = std::abs((cc.tail<1>() - nc.tail<1>())[0]) / v_max_yaw; // no pi check here, we might want to do multiple turns
        it->time_to_next = std::max(tp, ty) * 2;
//        std::cout<<"first guess: "<<it->time_to_next<<std::endl;
      }

      bool ret = optimize(vertices_, continuity, tol);

      if (!ret)
        return false;

      std::vector<double> time_scales = computeTimeScales(max_p, max_yaw);
      for (size_t is = 0; is < sx_.size(); ++is)
      {
        vertices_[is].time_to_next *= time_scales[is];

        if(time_multiplier > 0)
        {
          vertices_[is].time_to_next = ceil(vertices_[is].time_to_next / time_multiplier) * time_multiplier;
        }

//        std::cout<<"old time "<< vertices_[is].time_to_next/time_scale<<"new time "<< vertices_[is].time_to_next<<std::endl;
      }

      return optimize(vertices_, continuity, tol);
    }

    /**
     * \brief samples the whole path obtained by optimize() or optimizeWithTime() in steps of dt
     * \tparam n_p number of derivatives to sample for position. Implicitly obtained by result
     * \tparam n_y number of derivatives to sample for position. Implicitly obtained by result
     * \param[out] result vector with a Motion4D element for each sample
     * \param[in] dt timestep to sample
     */
    template<int n_p, int n_y>
      void sample(typename Motion4D<n_p, n_y>::Vector & result, double dt) const
      {
        double path_time = 0;

        for (typename SegmentVector::const_iterator it = sx_.begin(); it != sx_.end(); ++it)
        {
          path_time += it->t;
        }

        int n_samples = (path_time + dt * 0.5) / dt + 1;
        // hack around numerical badness ^^^^

        result.resize(n_samples);

        double t = 0;
        for (int i = 0; i < n_samples; i++)
        {
          Motion4D<n_p, n_y> &sample = result[i];
          samplePath(sample.x, sx_, t);
          samplePath(sample.y, sy_, t);
          samplePath(sample.z, sz_, t);
          samplePath(sample.yaw, syaw_, t);
          t += dt;
        }
      }

    /// returns the time-optimized vertices after a optimizeWithTime() call
    std::vector<Vertex4D> getVertices()
    {
      return vertices_;
    }

    /// returns a vector of polynomial segments for the x-axis
    SegmentVector getSegmentsX()
    {
      return sx_;
    }

    /// returns a vector of polynomial segments for the y-axis
    SegmentVector getSegmentsY()
    {
      return sy_;
    }

    /// returns a vector of polynomial segments for the z-axis
    SegmentVector getSegmentsZ()
    {
      return sz_;
    }

    /// returns a vector of polynomial segments for yaw
    SegmentVector getSegmentsYaw()
    {
      return syaw_;
    }
  };




} // end namespace path_planning

} // end namespace

#endif /* OPTIMIZATION_H_ */
