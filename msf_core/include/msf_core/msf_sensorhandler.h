/*

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

#ifndef MSF_SENSORHANDLER_H_
#define MSF_SENSORHANDLER_H_

namespace msf_core{
/**
 * \class SensorHandler
 * \brief handles a sensor driver which provides the sensor readings
 */
template<typename EKFState_T>
class SensorHandler {
  friend class MSF_SensorManager<EKFState_T> ;
  int lastseq_;
 protected:
  MSF_SensorManager<EKFState_T>& manager_;
  int sensorID;
  std::string topic_namespace_;
  std::string parameternamespace_;
  void setSensorID(int ID) {
    sensorID = ID;
  }
  void sequenceWatchDog(size_t seq, const std::string& topic) {
    if ((int) seq != lastseq_ + 1 && lastseq_ != 0) {
      MSF_WARN_STREAM(
          topic << ": message drop curr seq:" << seq << " expected: "
              << lastseq_ + 1);
    }
    lastseq_ = seq;
  }
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ;

  SensorHandler(MSF_SensorManager<EKFState_T>& mng,
                const std::string& topic_namespace, const std::string& parameternamespace)
      : lastseq_(0),
        manager_(mng),
        sensorID(-1),
        topic_namespace_(topic_namespace),
        parameternamespace_(parameternamespace) {
  }
  virtual ~SensorHandler() {
  }
};
}

#endif /* MSF_SENSORHANDLER_H_ */
