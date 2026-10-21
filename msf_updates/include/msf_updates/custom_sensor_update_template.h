#pragma once

#include <Eigen/Dense>

namespace msf_updates {

struct CustomSensorMeasurement {
  double time;
  Eigen::VectorXd z;
  Eigen::MatrixXd R;
};

class CustomSensorUpdate {
 public:
  CustomSensorUpdate() = default;

  void ProcessMeasurement(
      const CustomSensorMeasurement& measurement);

 private:
  void ComputeResidual();
  void ComputeJacobian();
};

}  // namespace msf_updates
