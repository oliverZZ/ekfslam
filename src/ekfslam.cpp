#include "ekfslam.h"

EKFSLAM::EKFSLAM() {
  is_initialized_ = false;
}

EKFSLAM(unsigned int landmark_size,
    unsigned int robot_pose_size,
    float _motion_noise) {

      N = landmark_size;
      int r = rob_pose_size;
      mu          = VectorXd::Zero(2*N + r, 1); // Full State Vector
      robSigma    = MatrixXd::Zero(r, r); // Covariance Matrix for robot state variables
      robMapSigma = MatrixXd::Zero(r, 2*N); // Covariance Matrix for robot to landmarks
      mapSigma    = INF*MatrixXd::Identity(2*N, 2*N); // Covariances of landmark positions wrt to each other
      Sigma       = MatrixXd::Zero(2*N + r, 2*N + r);// Full Covariance Matrix

      Sigma.topLeftCorner(r, r)          = robSigma;
      Sigma.topRightCorner(r, 2*N)       = robMapSigma;
      Sigma.bottomLeftCorner(2*N, r)     = robMapSigma.transpose();
      Sigma.bottomRightCorner(2*N, 2*N)  = mapSigma;

      float motion_noise =_motion_noise;

      R  = MatrixXd::Zero(2*N + r, 2*N + r); // Noise Matrix due to sensors
      R.topLeftCorner(3,3) << motion_noise, 0, 0,
                              0, motion_noise , 0,
                              0, 0,   motion_noise/10;
      observedLandmarks.resize(N);
      fill(observedLandmarks.begin(), observedLandmarks.end(), false);
}

void EKFSLAM::Prediction(const OdoReading& motion)

      double angle = mu(2);
      double r1    = motion.r1;
      double t     = motion.t;
      double r2    = motion.r2;

      MatrixXd Gt = MatrixXd(3,3);
      Gt << 1, 0, -t*sin(angle + r1),
             0, 1,  t*cos(angle + r1),
             0, 0,  1;
      MatrixXd Gt = MatrixXd::Zero(3+2*N, 3+2*N);

      //Gt.topLeftCorner(3,3) << Gtx;
      //Gt.bottomRightCorner(2*N, 2*N) << MatrixXd::Identity(2*N, 2*N);

      float c = cos(angle + r1);
      float s = sin(angle + r1);

      mu(0) = mu(0)  + t*c;
      mu(1) = mu(1) + t*s;
      mu(2) = mu(2) + r1 + r2;

      int size = Sigma.cols();
      Sigma.topLeftCorner(3,3) = Gt * Sigma.topLeftCorner(3,3) * Gt.transpose();
      Sigma.topRightCorner(3, size-3) = Gt * Sigma.topRightCorner(3, size-3);
      Sigma.bottomLeftCorner(size-3, 3) = Sigma.topRightCorner(3, size-3).transpose();
      Sigma = Sigma + R;
}

void EKFSLAM::Correction(const vector<LaserReading>& observation){
        int m = observation.size(); // number of











}
