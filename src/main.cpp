#include "../include/mapper.h"
#include "iostream"
#include "ekfslam.h"


int main(int argc, char* argv[]) {

  if (argc != 3) {
    //These are our usage instructions for inputing incorrect # of inputs
    std::cerr << "Files required are sensor.dat and world.dat";
    return 1;}

  Mapper mapper;
  mapper.initialize(argv[2]);


  return 0;
}
