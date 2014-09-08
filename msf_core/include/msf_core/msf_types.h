/*
 * Copyright (C) 2012-2013 Simon Lynen, ASL, ETH Zurich, Switzerland
 * You can contact the author at <slynen at ethz dot ch>
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
#ifndef MSF_TYPES_HPP_
#define MSF_TYPES_HPP_

#include <Eigen/Dense>
#include <type_traits>

#include <boost/shared_ptr.hpp>
#include  <memory> //std::shared_ptr
// Switch between boost shared and std shared:
using boost::shared_ptr;
using boost::dynamic_pointer_cast;

namespace msf_core {

typedef Eigen::Quaternion<double> Quaternion;

#define MSF_MAKE_EIGEN_TYPES(DIMENSION) \
    typedef Eigen::Matrix<double, DIMENSION, DIMENSION> Matrix##DIMENSION; \
    typedef Eigen::Matrix<double, DIMENSION, 1> Vector##DIMENSION;

MSF_MAKE_EIGEN_TYPES(1)
MSF_MAKE_EIGEN_TYPES(2)
MSF_MAKE_EIGEN_TYPES(3)
MSF_MAKE_EIGEN_TYPES(4)
MSF_MAKE_EIGEN_TYPES(5)
MSF_MAKE_EIGEN_TYPES(6)
MSF_MAKE_EIGEN_TYPES(7)
MSF_MAKE_EIGEN_TYPES(8)
MSF_MAKE_EIGEN_TYPES(9)

namespace constants{
  const static Vector3 GRAVITY((Vector3() << 0, 0, 9.80834).finished());  // At 47.37 lat (Zurich).
  const static double INVALID_TIME = -1;
  const static int INVALID_SEQUENCE = -1;
  const static int INVALID_ID = -1;
}

}
#endif  // MSF_TYPES_HPP_
