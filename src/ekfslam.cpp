#include "ekfslam.h"

EKFSLAM::EKFSLAM() {

}

EKFSLAM::EKFSLAM(unsigned int landmark_size, unsigned int robot_pose_size, float _motion_noise) {
      //This function creates mu, robotSigma, mapSigma and other thigns in the big matrx

      int N = landmark_size;
      int r = robot_pose_size;
      this->mu          = VectorXd::Zero(2*N + r); // Full State Vector
      this->robotSigma    = MatrixXd::Zero(r, r); // Covariance Matrix for robot state variables
      this->robMapSigma = MatrixXd::Zero(r, 2*N); // Covariance Matrix for robot to landmarks
      this->mapSigma    = INF*MatrixXd::Identity(2*N, 2*N); // Covariances of landmark positions wrt to each other
      this->Sigma       = MatrixXd::Zero(2*N + r, 2*N + r);// Full Covariance Matrix

      Sigma.topLeftCorner(r, r)          = robotSigma;
      Sigma.topRightCorner(r, 2*N)       = robMapSigma;
      Sigma.bottomLeftCorner(2*N, r)     = robMapSigma.transpose();
      Sigma.bottomRightCorner(2*N, 2*N)  = mapSigma;

      float motion_noise =_motion_noise;

      R  = MatrixXd::Zero(2*N + r, 2*N + r); // Noise Matrix due to montions
      R.topLeftCorner(3,3) << motion_noise, 0, 0,
                              0, motion_noise , 0,
                              0, 0,   motion_noise/10;
      observedLandmarks.resize(N); // create the matrix and initialize with all false. No landmark seen at the begining
      fill(observedLandmarks.begin(), observedLandmarks.end(), false);
}



void EKFSLAM::Prediction(const OdoReading& motion) {
  // This function predict the trace of robot according to odoReading

      double angle = this->mu(2);
      double r1    = motion.r1; //first rotation
      double r2    = motion.r2;//second rotation
      double t     = motion.t; //transition

      float c = cos(angle + r1);
      float s = sin(angle + r1);

      this->mu(0) = mu(0) + t*c;
      this->mu(1) = mu(1) + t*s;
      this->mu(2) = mu(2) + r1 + r2; //update state vector mu

      MatrixXd Gxt = MatrixXd(3,3); // Jacobian of motion
      Gxt << 1, 0, -t*sin(angle + r1),
             0, 1,  t*cos(angle + r1),
             0, 0,  1;

      int size = Sigma.cols();
      Sigma.topLeftCorner(3,3) = Gxt * Sigma.topLeftCorner(3,3) * Gxt.transpose();
      Sigma.topRightCorner(3, size-3) = Gxt * Sigma.topRightCorner(3, size-3);
      Sigma.bottomLeftCorner(size-3, 3) = Sigma.topRightCorner(3, size-3).transpose();
      Sigma = Sigma + R; // update covariance matrices
      this->Sigma = Sigma;

}

void EKFSLAM::Correction(const vector<LaserReading>& observation){
        //This functions update K, sigma and mu according to sensor_info
        int m = observation.size(); // number of measurements
        //int N = observedLandmarks.size();
        int sigma_rows = this->Sigma.rows();
        Eigen::MatrixXd Sigma = this->getSigma();

        Eigen::VectorXd pos_mu;
        Eigen::VectorXd robot_mu;
        pos_mu   = mu;
        robot_mu = mu;


        Eigen::MatrixXd H;             // = MatrixXd::Zero(5, 2*N + 3); //observation Jacobian.
        Eigen::MatrixXd Q              = MatrixXd::Identity(2,2)*0.01; // sensor noise matrix
        Eigen::VectorXd Z              = VectorXd::Zero(2);
        Eigen::VectorXd expectedZ      = VectorXd::Zero(2);

        Eigen::MatrixXd LowH(2,5);


        for (int i = 0; i < m; i++) {
            auto&     reading = observation[i];
            int       landmarkId = reading.id;
            float     range      = reading.range;
            float     bearing    = reading.bearing;
            Z(0) = range;
            Z(1) = bearing;
            Eigen::MatrixXd Fxj = MatrixXd::Zero(5, sigma_rows);
            Fxj.block<3,3>(0,0) << 1,0,0, // create Fxj matrix
                                   0,1,0,
                                   0,0,1;
            Fxj.block<2,2>(3,2*landmarkId+1) << 1,0,
                                                0,1;

            //landmark is not seen before, so to initialize the landmarks
            if (this->observedLandmarks[landmarkId-1]==false) {
                pos_mu(0) = robot_mu(0) + range*cos(mu(2) + robot_mu(2));
                pos_mu(1) = robot_mu(1) + range*sin(mu(2) + robot_mu(2));

                //Indicate in the observedLandmarks vector that this landmark has been observed
                robot_mu(landmarkId*2+1) = pos_mu(0);
                robot_mu(landmarkId*2+2) = pos_mu(1);

                observedLandmarks[landmarkId-1] = true;
              }
              else{
                pos_mu(0) = robot_mu(landmarkId*2+1);
                pos_mu(1) = robot_mu(landmarkId*2+2);
              }

             double Deltax = pos_mu(0) - robot_mu(0);// delta x
             double Deltay = pos_mu(1) - robot_mu(1);//delta y
             double q = pow(Deltax, 2) + pow(Deltay, 2);
             expectedZ(0) = sqrt(q);
             expectedZ(1) = atan2(Deltay, Deltax) - robot_mu(2);
             LowH << -sqrt(q)*Deltax/q, -sqrt(q)*Deltay/q, 0, sqrt(q)*Deltax/q, sqrt(q)*Deltay/q,
                      Deltay/q,         -1 * Deltax/q,  -q/q, -1*Deltay/q,      Deltax/q;
              H = LowH * Fxj;


              Eigen::MatrixXd Ht = H.transpose();
              Eigen::MatrixXd HQ = (H*Sigma*Ht) + Q;
              Eigen::MatrixXd Si = HQ.inverse();
              Eigen::MatrixXd K = Sigma*Ht*Si;
              Eigen::VectorXd diff = Z - expectedZ;


                //for (int j = 1; j < diff.size(); j += 2) {
                        while(diff(1)>M_PI) {
                          diff(1) = diff(1) - 2*M_PI;
                        }
                        while(diff(1)<M_PI) {
                          diff(1) = diff(1) + 2*M_PI;
                        }
                  //}
              robot_mu = robot_mu + K * diff;
              while(robot_mu(2)>M_PI) {
                  robot_mu(2) = robot_mu(2) - 2*M_PI;
                }
                while(robot_mu(2)<M_PI) {
                  robot_mu(2) = robot_mu(2) + 2*M_PI;
                }

              Sigma = Sigma - K*H*Sigma;

        }

        this->mu = robot_mu;
        this->Sigma = Sigma;
      }
