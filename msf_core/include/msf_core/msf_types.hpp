/*
 *  Created on: Nov 7, 2012
 *      Author: slynen
 */

#ifndef MSF_TYPES_HPP_
#define MSF_TYPES_HPP_

#include <Eigen/Dense>

namespace msf_core{
//typedefs
typedef const Eigen::Matrix<double, 3, 3> ConstMatrix3;
typedef Eigen::Matrix<double, 3, 3> Matrix3;
typedef const Eigen::Matrix<double, 4, 4> ConstMatrix4;
typedef Eigen::Matrix<double, 4, 4> Matrix4;
typedef const Eigen::Matrix<double, 3, 1> ConstVector3;
typedef Eigen::Vector3d Vector3;


}
#endif /* MSF_TYPES_HPP_ */
