/*
 * Copyright (c) 2012, Markus Achtelik, ASL, ETH Zurich, Switzerland
 * You can contact the author at <acmarkus at ethz dot ch>
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
#include <msf_core/gps_conversion.h>
#include <msf_core/msf_macros.h>
#include <ros/ros.h>
#include <msf_core/sincos.h>

namespace msf_core {

GPSConversion::GPSConversion()
  : ecef_ref_orientation_(),
    ecef_ref_point_(msf_core::Vector3::Zero())
{
  ecef_ref_orientation_.setIdentity();
}

GPSConversion::~GPSConversion() {}

void GPSConversion::InitReference(const double latitude,
                                  const double longitude,
                                  const double altitude) {
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

  ecef_ref_point_ = WGS84ToECEF(latitude, longitude, altitude);
}

msf_core::Vector3 GPSConversion::WGS84ToECEF(const double latitude,
                                             const double longitude,
                                             const double altitude) const {
  const double a = 6378137.0;  // semi-major axis
  const double e_sq = 6.69437999014e-3;  // first eccentricity squared

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

msf_core::Vector3 GPSConversion::ECEFToENU(const msf_core::Vector3& ecef) const {

  if (ecef_ref_point_.norm() == 0) {
    MSF_ERROR_STREAM_ONCE(
        "The gps reference is not initialized. Returning global coordinates. This warning will only show once.");
  }
  return ecef_ref_orientation_ * (ecef - ecef_ref_point_);
}

msf_core::Vector3 GPSConversion::WGS84ToENU(const double latitude,
                                            const double longitude,
                                            const double altitude) const {
    const msf_core::Vector3 ecef = this->WGS84ToECEF(latitude, longitude, altitude);
    return this->ECEFToENU(ecef);
}

void GPSConversion::AdjustReference(const double z_correction) {

  MSF_WARN_STREAM("z-ref old: "<< ecef_ref_point_(2));
  ecef_ref_point_(2) += z_correction;
  MSF_WARN_STREAM("z-ref new: "<< ecef_ref_point_(2));
}

}
