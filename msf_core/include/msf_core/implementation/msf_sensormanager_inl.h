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
#ifndef MSF_SENSORHANDLER_INL_H_
#define MSF_SENSORHANDLER_INL_H_
#include <msf_core/msf_sensorhandler.h>
#include <msf_core/msf_types.h>
#include <msf_core/msf_core.h>

namespace msf_core {
template<typename EKFState_T>
MSF_SensorManager<EKFState_T>::MSF_SensorManager() {
  sensorID_ = 0;
  data_playback_ = false;
  //TODO (slynen): Make this a (better) design. This is so aweful.
  msf_core_.reset(new msf_core::MSF_Core<EKFState_T>(*this));
}
}  // namespace msf_core
#endif  // MSF_SENSORHANDLER_INL_H_
