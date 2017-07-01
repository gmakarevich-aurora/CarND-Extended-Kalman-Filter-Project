#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class Tools {
public:
  /**
  * Constructor.
  */
  Tools();

  /**
  * Destructor.
  */
  virtual ~Tools();

  /**
  * A helper method to calculate RMSE.
  */
  VectorXd CalculateRMSE(
      const vector<VectorXd> &estimations,
      const vector<VectorXd> &ground_truth) const;

  /**
  * A helper method to calculate Jacobians.
  */
  MatrixXd CalculateJacobian(const VectorXd& x_state) const;

  // A helper method to convert Radar measurements into General measurements.
  VectorXd ConvertRadarMeasurementToGeneral(const VectorXd& measurement) const;

  // A helper method to convert General measurements into Radar measurements.
  VectorXd ConvertGeneralMeasurementToRadar(const VectorXd& measurement) const;
};

#endif /* TOOLS_H_ */
