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
  static constexpr double defaultRejectionDivergenceThreshold_ = 100.0;
  static constexpr double upperNoiseLimit_ = 0.6;
  static constexpr double lowerNoiseLimit_ = 0.2;
  static constexpr double desiredNoiseLevel_ = 0.3;
  static constexpr int rovioResetSaveTime = 3;
  static constexpr int minRequiredInitPoints = 30;
  static constexpr double minRequiredInitMovement = 2.0;
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
  //params for outlierrejection
  bool enable_mah_outlier_rejection_;
  double mah_threshold_;
  //params for noise estimation
  bool enable_noise_estimation_;
  double running_maha_dist_average_;
  double average_discount_factor_;
  //params for divergence recovery
  bool enable_divergence_recovery_;
  double n_rejected_;
  double n_curr_rejected_;
  double n_accepted_;
  double rejection_divergence_threshold_;
  double max_noise_threshold_;


  void SetSensorID(int ID) {
    sensorID = ID;
  }

  int GetSensorID(){
    return sensorID;
  }
  void SequenceWatchDog(size_t seq, const std::string& topic) {
    if (static_cast<int>(seq) != lastseq_ + 1 && lastseq_ != 0) {
      MSF_WARN_STREAM(
          topic << ": message drop curr seq:" << seq << " expected: "
                << lastseq_ + 1);
    }
    lastseq_ = seq;
  }
  bool InitPointsReady()
  {
    //conditions for ready
    if(init_points_.size()>minRequiredInitPoints && (init_points_[0].head(3)-init_points_[init_points_.size()-1].head(3)).norm()>minRequiredInitMovement)
    {
      //points are not allowed to be on a straight line:
      //use QR and check residual to see wether can be approximated by a line or not
      //does PCA as described in https://math.stackexchange.com/questions/1611308/best-fit-line-with-3d-points
      //1/n X^T*P*X is the covariance of the points
      //P is 1-1/n on the diagonal and -1/n elsewhere
      const int npoints=init_points_.size();
      Eigen::MatrixXd P = Eigen::MatrixXd::Constant(npoints, npoints, -1.0/npoints);
      P.diagonal() = Eigen::ArrayXd::Constant(npoints, 1.0-1.0/npoints);
      //build X matrix
      Eigen::MatrixXd X(1,1);
      X.resize(npoints, 3);
      for (int i=0; i<npoints;++i)
      {
        X.row(i)=init_points_[i].head(3);
      }
      //now the largest eigenvalue gives the "length" of the line (with its corresponding eigenvector as direction) [not interested in that]
      //the second eigenvalue is ~the error
      //to have it a relative good 2d movement maybe use some constraint on 2nd largest/largest (think about this)
      Eigen::VectorXd eigenvalues = (X.transpose()*P*X).eigenvalues().real();
      std::sort(eigenvalues.data(), eigenvalues.data()+eigenvalues.size(), std::greater<double>());
      //now should be sorted
      if(eigenvalues(1)>=0.1) 
      {
        return true;
      }
    }
    return false;  
  }
public:
  //vars for stable init
  bool collect_for_init_;
  bool ready_for_init_;
  //data sturcture for collecting points for stable init (if runtime is problematic may want to use something better)
  //each vector saves: x,y,z,t
  std::vector<Eigen::Vector4d> init_points_;
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SensorHandler(MSF_SensorManager<EKFState_T>& mng,
                const std::string& topic_namespace,
                const std::string& parameternamespace)
      : lastseq_(0),
        manager_(mng),
        sensorID(constants::INVALID_ID),
        topic_namespace_(topic_namespace),
        parameternamespace_(parameternamespace),
        received_first_measurement_(false),
        n_rejected_(0), n_curr_rejected_(0),
        n_accepted_(0), collect_for_init_(false),
        ready_for_init_(false){
  }
  virtual ~SensorHandler() {
  }
  bool ReceivedFirstMeasurement() const {return received_first_measurement_;}
  double GetMaxNoiseThreshold() const {return max_noise_threshold_;}
};
}
#endif  // MSF_SENSORHANDLER_H_
