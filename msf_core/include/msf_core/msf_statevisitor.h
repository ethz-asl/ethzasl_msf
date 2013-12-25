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
#ifndef MSF_STATEVISITOR_H_
#define MSF_STATEVISITOR_H_

namespace msf_core {
/**
 * \brief Visitor pattern to allow the user to set state init values.
 */
template<typename EKFState_T>
class StateVisitor {
 public:
  /**
   * \brief The state is set to zero/identity, this method will be called to
   * give the user the possibility to change the reset values of some states.
   */
  virtual void ResetState(EKFState_T& state) const = 0;
  virtual ~StateVisitor() {
  }
  ;
};
}
#endif  // MSF_STATEVISITOR_H_
