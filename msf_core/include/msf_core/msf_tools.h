/*

 Copyright (c) 2012, Simon Lynen, ASL, ETH Zurich, Switzerland
 You can contact the author at <slynen at ethz dot ch>

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

#ifndef MSF_TOOLS_H_
#define MSF_TOOLS_H_

#include <Eigen/Dense>
#include <algorithm>

namespace msf_core {
/***
 * Computes the median of a given vector.
 */
template<typename D>
typename Eigen::MatrixBase<D>::Scalar getMedian(
    const Eigen::MatrixBase<D> & data) {
  static_assert(
      (Eigen::MatrixBase<D>::ColsAtCompileTime == 1),
      "getMedian only takes Eigen column vectors as arguments");
  Eigen::Matrix<typename Eigen::MatrixBase<D>::Scalar,
      Eigen::MatrixBase<D>::RowsAtCompileTime,
      // Copy so we don't sort the original vector.
      Eigen::MatrixBase<D>::ColsAtCompileTime> m = data;

  if (Eigen::MatrixBase<D>::SizeAtCompileTime) {
    double * begin = m.data();
    double * end = m.data() + m.SizeAtCompileTime;
    double * middle = begin + static_cast<int>(std::floor((end - begin) / 2));
    std::nth_element(begin, middle, end);
    return *middle;
  } else
    return 0;
}

/***
 * Outputs the time in seconds in a human readable format for debugging.
 */
double timehuman(double val);

}

#endif  // MSF_TOOLS_H_
