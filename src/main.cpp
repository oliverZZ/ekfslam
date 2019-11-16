#include "../include/mapper.h"
#include "iostream"
#include "ekfslam.h"
#include "../include/sensor_info.h"


int main(int argc, char* argv[]) {

  if (argc != 3) {
    //These are our usage instructions for inputing incorrect # of inputs
    std::cerr << "Files required are sensor.dat and world.dat";
    return 1;}

  Mapper mapper;
  mapper.initialize(argv[2]);

  MeasurementPackage sensor_info;
  sensor_info.initialize(argv[1]);

  


  return 0;
}
