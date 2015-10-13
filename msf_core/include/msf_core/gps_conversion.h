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
#ifndef GPS_CONVERSION_H_
#define GPS_CONVERSION_H_

#include <Eigen/Dense>
#include <msf_core/msf_types.h>

namespace msf_core {
class GPSConversion {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GPSConversion();
  // Default copy constructor and assignment operator are OK.

  ~GPSConversion();

  void InitReference(const double latitude,
                     const double longitude,
                     const double altitude);

  msf_core::Vector3 WGS84ToECEF(const double latitude,
                                const double longitude,
                                const double altitude) const;

  msf_core::Vector3 ECEFToENU(const msf_core::Vector3& ecef) const;

  // Simply calls WGS84ToECEF(...) followed by ECEFToENU(...)
  msf_core::Vector3 WGS84ToENU(const double latitude,
                               const double longitude,
                               const double altitude) const;

  void AdjustReference(const double z_correction);

private:
  msf_core::Quaternion ecef_ref_orientation_;
  msf_core::Vector3 ecef_ref_point_;
};
}  // namespace msf_core
#endif  // GPS_CONVERSION_H_
