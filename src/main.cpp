#include "../include/mapper.h"
#include "iostream"
#include "ekfslam.h"
#include "../include/sensor_info.h"
#include "../include/plotter.h"

int main(int argc, char* argv[]) {
  if (argc != 3) {
    //These are our usage instructions for inputing incorrect # of inputs
    std::cerr << "Files required are sensor.dat and world.dat";
    return 1;}
  Mapper mapper;
  MeasurementPackage sensor_info;
  Draw ourPlot;


  mapper.initialize(argv[2]);
  sensor_info.initialize(argv[1]);
  int sensorSize = sensor_info.data.size();
  int mapperSize = mapper.data.size();
  EKFSLAM ekfslam(mapperSize);

  //Loop through all sensor data
  for (int i = 0; i < sensorSize; i++){
        ourPlot.Clear();
        OdoReading odoReading = sensor_info.data[i].odo;
        std::vector<LaserReading> scans = sensor_info.data[i].scans;
        ekfslam.Prediction(odoReading);
        ekfslam.Correction(scans);
        VectorXd mu = ekfslam.getMu();
        MatrixXd Sigma = ekfslam.getSigma();
        vector<bool> observedLandmarks = ekfslam.getObservedLandmarks();

        ourPlot.Plot_State(mu,Sigma,mapper,observedLandmarks,scans);
        ourPlot.Pause();
        //stringstream ss;
        //ss << setfill('0') << setw(3) << i;
        //ourPlot.Save("../src/"+ss.str());
  }
  //Visualization with Draw class
  // for length of data
  // Plot: Plot_State(const VectorXd& mu, const MatrixXd& sigma,
          //const Mapper& mapper, const vector<bool>&observedLandmarks,
          //const vector<LaserReading>& Z)
          ourPlot.Show();
  return 0;
}
