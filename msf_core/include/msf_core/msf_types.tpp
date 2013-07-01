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

#ifndef MSF_TYPES_HPP_
#define MSF_TYPES_HPP_

#include <Eigen/Dense>

#include <boost/shared_ptr.hpp>
#include  <memory> //std::shared_ptr

//switch between boost shared and std shared
using boost::shared_ptr;
using boost::dynamic_pointer_cast;

namespace msf_core{

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


}
#endif /* MSF_TYPES_HPP_ */
