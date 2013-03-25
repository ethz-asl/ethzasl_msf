/*

Copyright (c) 2013, Markus Achtelik, ASL, ETH Zurich, Switzerland
You can contact the author at <acmarkus at ethz dot ch>
Copyright (c) 2013, Simon Lynen, ASL, ETH Zurich, Switzerland
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

#include <msf_core/gps_conversion.h>
#include <msf_core/msf_macros.h>
#include <ros/ros.h>

namespace msf_core{

GPSConversion::GPSConversion(){
  ecef_ref_point_ = msf_core::Vector3::Zero();
  ecef_ref_orientation_.setIdentity();
}

void GPSConversion::initReference(const double & latitude, const double & longitude, const double & altitude)
{
  msf_core::Matrix3 R;
  double s_long, s_lat, c_long, c_lat;
  sincos(latitude * DEG2RAD, &s_lat, &c_lat);
  sincos(longitude * DEG2RAD, &s_long, &c_long);

  R(0, 0) = -s_long;
  R(0, 1) = c_long;
  R(0, 2) = 0;

  R(1, 0) = -s_lat * c_long;
  R(1, 1) = -s_lat * s_long;
  R(1, 2) = c_lat;

  R(2, 0) = c_lat * c_long;
  R(2, 1) = c_lat * s_long;
  R(2, 2) = s_lat;

  ecef_ref_orientation_ = msf_core::Quaternion(R);

  ecef_ref_point_ = wgs84ToEcef(latitude, longitude, altitude);
}

msf_core::Vector3 GPSConversion::wgs84ToEcef(const double & latitude, const double & longitude, const double & altitude) const
{
  const double a = 6378137.0; // semi-major axis
  const double e_sq = 6.69437999014e-3; // first eccentricity squared

  double s_long, s_lat, c_long, c_lat;
  sincos(latitude * DEG2RAD, &s_lat, &c_lat);
  sincos(longitude * DEG2RAD, &s_long, &c_long);

  const double N = a / sqrt(1 - e_sq * s_lat * s_lat);

  msf_core::Vector3 ecef;

  ecef[0] = (N + altitude) * c_lat * c_long;
  ecef[1] = (N + altitude) * c_lat * s_long;
  ecef[2] = (N * (1 - e_sq) + altitude) * s_lat;

  return ecef;
}

msf_core::Vector3 GPSConversion::ecefToEnu(const msf_core::Vector3 & ecef) const
{
  if(ecef_ref_point_.norm() == 0){
    ROS_ERROR_STREAM_ONCE("The gps reference is not initialized. Returning global coordinates. This warning will only show once.");
  }
  return ecef_ref_orientation_ * (ecef - ecef_ref_point_);
}

void GPSConversion::adjustReference(double z_corr){
  ROS_WARN_STREAM("z-ref old: "<<ecef_ref_point_(2));
  ecef_ref_point_(2) += z_corr;
  ROS_WARN_STREAM("z-ref new: "<<ecef_ref_point_(2));
}

}
