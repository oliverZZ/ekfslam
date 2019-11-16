#include "ekfslam.h"

EKFSLAM::EKFSLAM() {
  is_initialized_ = false;
}

void EKFSLAM(unsigned int landmark_size,
    unsigned int robot_pose_size = 3,
    float _motion_noise = 0.1) {
      Eigen::MatrixXd robSigma;
      Eigen::MatrixXd robMapSigma;
      Eigen::MatrixXd Sigma;
      Eigen::MatrixXd mapSigma;

      
      int N = landmark_size;
      int r = rob_pose_size;
      mu          = VectorXd::Zero(2*N + r, 1);
      robSigma    = MatrixXd::Zero(r, r);
      robMapSigma = MatrixXd::Zero(r, 2*N);
      mapSigma    = INF*MatrixXd::Identity(2*N, 2*N);
      Sigma       = MatrixXd::Zero(2*N + r, 2*N + r);
}
