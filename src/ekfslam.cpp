#include "ekfslam.h"

EKFSLAM::EKFSLAM() {

}

EKFSLAM::EKFSLAM(unsigned int landmark_size, unsigned int robot_pose_size, float _motion_noise) {

      int N = landmark_size;
      int r = robot_pose_size;
      mu          = VectorXd::Zero(2*N + r); // Full State Vector
      robotSigma    = MatrixXd::Zero(r, r); // Covariance Matrix for robot state variables
      robMapSigma = MatrixXd::Zero(r, 2*N); // Covariance Matrix for robot to landmarks
      mapSigma    = INF*MatrixXd::Identity(2*N, 2*N); // Covariances of landmark positions wrt to each other
      Sigma       = MatrixXd::Zero(2*N + r, 2*N + r);// Full Covariance Matrix

      Sigma.topLeftCorner(r, r)          = robotSigma;
      Sigma.topRightCorner(r, 2*N)       = robMapSigma;
      Sigma.bottomLeftCorner(2*N, r)     = robMapSigma.transpose();
      Sigma.bottomRightCorner(2*N, 2*N)  = mapSigma;

      float motion_noise =_motion_noise;

      R  = MatrixXd::Zero(2*N + r, 2*N + r); // Noise Matrix due to montions
      R.topLeftCorner(3,3) << motion_noise*3, 0, 0,
                              0, motion_noise*3 , 0,
                              0, 0,   motion_noise*2;
      observedLandmarks.resize(N);
      fill(observedLandmarks.begin(), observedLandmarks.end(), false);
}



void EKFSLAM::Prediction(const OdoReading& motion) {

      double angle = normalized(mu(2));
      double r1    = normalized(motion.r1); //first rotation
      double r2    = normalized(motion.r2);//second rotation
      double t     = motion.t; //transition

      float c = cos(angle + r1);
      float s = sin(angle + r1);

      mu(0) = mu(0) + t*c;
      mu(1) = mu(1) + t*s;
      mu(2) = normalized(mu(2)) + r1 + r2; //update state vector mu

      MatrixXd Gxt = MatrixXd(3,3); // Jacobian of motion
      Gxt << 1, 0, -t*sin(angle + r1),
             0, 1,  t*cos(angle + r1),
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
        int sigma_rows = Sigma.rows();
        Eigen::VectorXd pos_mu;
        Eigen::VectorXd* robot_mu;
        pos_mu   = mu;
        robot_mu = &mu;


        Eigen::MatrixXd H              = MatrixXd::Zero(5, 2*N + 3); //observation Jacobian. Size is the same as Fx,j
        Eigen::MatrixXd Q              = MatrixXd::Identity(2,2)*0.1; // sensor noise matrix
        Eigen::VectorXd Z              = VectorXd::Zero(2);
        Eigen::VectorXd expectedZ      = VectorXd::Zero(2);

        Eigen::MatrixXd LowH           = MatrixXd::Zero(2,5);


        for (int i = 0; i < m; i++) {
            auto&     reading = observation[i];
            int       landmarkId = reading.id;
            float     range      = reading.range;
            float     bearing    = normalized(reading.bearing);
            Z(0) = range;
            Z(1) = bearing;
            Eigen::MatrixXd Fxj            = MatrixXd::Zero(5, sigma_rows);
            Fxj.block<3,3>(0,0) << 1,0,0,
                                   0,1,0,
                                   0,0,1;
            Fxj.block<2,2>(3,2*landmarkId+1) << 1,0,
                                                0,1;
            //landmark is not seen before, so to initialize the landmarks
            if (!observedLandmarks[landmarkId-1]) {
                pos_mu(0) = (*robot_mu)[0] + range*cos(normalized(mu(2)) + normalized((*robot_mu)[2]));
                pos_mu(1) = (*robot_mu)[1] + range*sin(normalized(mu(2)) + normalized((*robot_mu)[2]));

                //Indicate in the observedLandmarks vector that this landmark has been observed
                (*robot_mu)[landmarkId*2+1] = pos_mu(0);
                (*robot_mu)[landmarkId*2+2] = pos_mu(1);

                observedLandmarks[landmarkId-1] = true;
              }
              else{
                pos_mu(0) = (*robot_mu)[landmarkId*2+1];
                pos_mu(1) = (*robot_mu)[landmarkId*2+2];
              }
             //Eigen::MatrixXd Delta = MatrixXd::Zero(2,1);
             double Deltax = pos_mu(0) - (*robot_mu)[0];// delta x
             double Deltay = pos_mu(1) - (*robot_mu)[1];//delta y
			 
			 if(Deltax > range || Deltay > range)
			 {
				cout << "range        = " << range << "\n";
				cout << "deltaX       = " << Deltax << "\n";//----------------------DELETEME
				cout << "deltaY       = " << Deltay << "\n";//-----------------------DELETEME
				cout << "calc Range = " << sqrt(Deltax*Deltax + Deltay*Deltay) << "\n";
				cout << "range diff   = " << range - sqrt(Deltax*Deltax + Deltay*Deltay) << "\n\n";
			 }
             double q = pow(Deltax, 2) + pow(Deltay, 2);
             expectedZ(0) = sqrt(q);
             expectedZ(1) = normalized(atan2(Deltay, Deltax) - normalized((*robot_mu)[2]));
             LowH << -sqrt(q)*Deltax/q, -sqrt(q)*Deltay/q, 0, sqrt(q)*Deltax/q, sqrt(q)*Deltay/q,
                      Deltay/q,         -1 * Deltax/q,  -q/q, -1*Deltay/q,      Deltax/q;
              H = LowH * Fxj;

              Eigen::MatrixXd Ht = H.transpose();
              Eigen::MatrixXd HQ = (H*Sigma*Ht) + Q;
              Eigen::MatrixXd Si = HQ.inverse();
              Eigen::MatrixXd K = Sigma*Ht*Si;
              Eigen::VectorXd diff = Z - expectedZ;
			 (*robot_mu) = (*robot_mu) + K * diff;
				/*
                for (int j = 1; j < diff.size(); j += 2) {
                        while(diff(j)>M_PI) {
                          diff(j) = diff(j) - M_PI;
                        }
                        while(diff(j)<M_PI) {
                          diff(j) = diff(j) + M_PI;
                        }
                  }
             
              while(robot_mu(2)>M_PI) {
                  robot_mu(2) = robot_mu(2) - M_PI;
                }
                while(robot_mu(2)<M_PI) {
                  robot_mu(2) = robot_mu(2) + M_PI;
                }
				*/
              Sigma = Sigma - K*H*Sigma;

        }

        //mu = (*robot_mu); -----------------not needed since updated by reference
      }
