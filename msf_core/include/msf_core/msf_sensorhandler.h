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
#ifndef MSF_SENSORHANDLER_H_
#define MSF_SENSORHANDLER_H_

namespace msf_core {

  static constexpr double kDefaultMahThreshold_ = 100.0;
/**
 * \class SensorHandler
 * \brief Handles a sensor driver which provides the sensor readings.
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
  bool received_first_measurement_;
  bool enable_mah_outlier_rejection_;
  double mah_threshold_;

  void SetSensorID(int ID) {
    sensorID = ID;
  }
  void SequenceWatchDog(size_t seq, const std::string& topic) {
    if (static_cast<int>(seq) != lastseq_ + 1 && lastseq_ != 0) {
      MSF_WARN_STREAM(
          topic << ": message drop curr seq:" << seq << " expected: "
                << lastseq_ + 1);
    }
    lastseq_ = seq;
  }
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SensorHandler(MSF_SensorManager<EKFState_T>& mng,
                const std::string& topic_namespace,
                const std::string& parameternamespace)
      : lastseq_(0),
        manager_(mng),
        sensorID(constants::INVALID_ID),
        topic_namespace_(topic_namespace),
        parameternamespace_(parameternamespace),
        received_first_measurement_(false) {
  }
  virtual ~SensorHandler() {
  }
  bool ReceivedFirstMeasurement() const {return received_first_measurement_;}
};
}
#endif  // MSF_SENSORHANDLER_H_
