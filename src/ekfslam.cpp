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

      R  = MatrixXd::Zero(2*N + r, 2*N + r); // Noise Matrix due to montions
      R.topLeftCorner(3,3) << motion_noise, 0, 0,
                              0, motion_noise , 0,
                              0, 0,   motion_noise/10;
      observedLandmarks.resize(N);
      fill(observedLandmarks.begin(), observedLandmarks.end(), false);
}

void EKFSLAM::Prediction(const OdoReading& motion)

      double angle = mu(2);
      double r1    = motion.r1; //first rotation
      double r2    = motion.r2;//second rotation
      double t     = motion.t; //transition

      float c = cos(angle + r1);
      float s = sin(angle + r1);

      mu(0) = mu(0) + t*c;
      mu(1) = mu(1) + t*s;
      mu(2) = mu(2) + r1 + r2; //update state vector mu

      MatrixXd Gxt = MatrixXd(3,3); // Jacobian of motion
      Gxt << 1, 0, -t*cos(angle + r1),
             0, 1,  t*sin(angle + r1),
             0, 0,  1;

      int size = Sigma.cols();
      Sigma.topLeftCorner(3,3) = Gxt * Sigma.topLeftCorner(3,3) * Gxt.transpose();
      Sigma.topRightCorner(3, size-3) = Gxt * Sigma.topRightCorner(3, size-3);
      Sigma.bottomLeftCorner(size-3, 3) = Sigma.topRightCorner(3, size-3).transpose();
      Sigma = Sigma + R; // update covariance matrices
}

void EKFSLAM::Correction(const vector<LaserReading>& observation){
        int m = observation.size(); // number of measurements
        int N = observedLandmarks.size();
        MatrixXd H = MatrixXd::Zero(5, 2*N + 3); //observation Jacobian. Size is the same as Fx,j
        MatrixXd Q = MatrixXd::Identity(5,5)*0.01; // sensor noise matrix
        MatrixXd Z          = MatrixXd::Zero(2*m);
        MatrixXd expectedZ  = MatrixXd::Zero(2*m);
        MatrixXd Fxj = MatrixXd::Zero(5, 2*N+3);
        MatrixXd LowH = MatrixXd::Zero(2,5);
        Fxj.block<3,3>(0,0) << 1,0,0,
                               0,1,0,
                               0,0,1;
        Fxj.block<2,2>(3,2*m) << 1,0,
                                 0,1;

        for (int i = 0; i < m; i++) {
            auto&     reading = observation[i];
            int       landmarkId = reading.id;
            float     range      = reading.range;
            float     bearing    = reading.bearing;
            Z(2*i) = range;
            Z(2*i+1) = bearing;
            //landmark is not seen before, so to initialize the landmarks
            if (!observedLandmarks[landmarkId-1]) {
                mu(2*landmarkId+1) = mu(0) + range*cos(mu(2) + bearing);
                mu(2*landmarkId+2) = mu(1) + range*sin(mu(2) + bearing);
                //Indicate in the observedLandmarks vector that this landmark has been observed
                observedLandmarks[landmarkId-1] = true;
                }
             MatrixXd Delta = MatrixXd::Zero(1,1);
             Delta(0) = mu(2*landmarkId+1) - mu(0);// delta x
             Delta(1) = mu(2*landmarkId+2) - mu(1);//delta y
             float q = Delta.transpose * Delta;
             expectedZ(2*i) = sqrt(q);
             expectedZ(2*i+1) = atan2(Delta(1), Delta(0)) - mu(2);
             LowH << -sqrt(q)*Delta(0)/q, -sqrt(q)*Delta(1)/q, 0, sqrt(q)*Delta(0)/q, sqrt(q)*Delta(1)/q,
                      Delta(1)/q,         -1 * Delta(0)/q,  -q/q, -1*Delta(1)/q,      Delta(0)/q;
              H = LowH * Fxj;
              MatrixXd Ht = H.transpose();
              MatrixXd HQ = (H*Sigma*Ht) + Q;
              MatrixXd Si = HQ.inverse();
              MatrixXd K = Sigma*Ht*Si;
              VectorXd diff = Z - expectedZ;
              tools.normalize_bearing(diff);
              mu = mu + K * diff;
              Sigma = Sigma - K*H*Sigma;
        }
      }
