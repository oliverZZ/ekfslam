#pragma once

#include "Eigen/Dense"
#include "./common.h"

// Struct Representing a standard Odometry reading
// Contains two rotations and one translation
struct OdoReading{
    float r1;
    float t;
    float r2;
};

// Struct representing a standard velocity reading
// Contains a linear and angular velocity
struct VelReading {
    float linearVel;
    float angularVel;
};

// Struct representing a reading from a laser scanner
// Each reading has an ID, a range, and a bearing
struct LaserReading {
    uint64_t id;
    float range;
    float bearing;
};

// Struct containing all sensor readings at one time Step
// One Odometry reading at each time step
// Vector of all laser scans at each time step
class Record {
 public:
    Record() {}
    OdoReading odo;
    vector<LaserReading> scans;
};

// Class to represent measured data
// data - vector containing all sensor readings at each time step
// initialize - read in sensor information from file with specfied path 
class MeasurementPackage {
 public:
  vector<Record> data;
  void initialize(const string& filename) {
    ifstream in_file(filename, ifstream::in);
    if (!in_file.is_open()) {
    cerr << "Cannot open input file: " << filename << endl;
      exit(EXIT_FAILURE);
    }

    string line;
    Record record;
    int index = 0;
    while (getline(in_file, line)) {
        string sensor_type;
        istringstream ss(line);
        ss >> sensor_type;
        // measurement type r1 t r2
        if (sensor_type.compare("ODOMETRY") == 0) {
            // end the first record;
            if (record.scans.size() != 0) {
                data.push_back(record);
                record.scans.clear();
                if (debug && index < 50)
                    cout << index << "-----------" << endl;
                index++;
            }
            auto& odo = record.odo;
            ss >> odo.r1;
            ss >> odo.t;
            ss >> odo.r2;
            if (debug && index < 50)
            cout << record.odo.r1 << ": " << record.odo.t << ": "
                << record.odo.r2 << endl;
      } else if (sensor_type.compare("SENSOR") == 0) {
          auto& scans = record.scans;
          LaserReading laserR;
          ss >> laserR.id;
          ss >> laserR.range;
          ss >> laserR.bearing;
          scans.push_back(laserR);
          if (debug && index < 50)
          cout << scans.back().id << ": " << scans.back().range << ": "
            << scans.back().bearing << endl;
      }
    }
    if (record.scans.size() != 0) {
        data.push_back(record);
    }
    if (in_file.is_open()) {
        in_file.close();
    }
  }
};
