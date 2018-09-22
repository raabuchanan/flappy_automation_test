#ifndef PIP_TRACKER_H_
#define PIP_TRACKER_H_

#include <iostream>
#include <vector>
#include <math.h>
#include <ros/console.h>

namespace flappy_navigation {


class PipeGapTracker {
  public:
	PipeGapTracker();
	~PipeGapTracker();

    void processUpdate(double x_displacement, double y_displacement);
    void measurementUpdate(double x_measurement, std::vector<double> y_measurements, double resolution);
    double getXpos();
    double getYpos();
    bool getInitialization();

  private:

  double reInitializeKalmanFilter();

  double x_pos_;
  double y_pos_;

  double x_var_;
  double y_var_;

  bool initialized_;

  double process_noise_ = 0.01;
  double measurement_noise_ = 0.1; // should be related to resolution and distance

};

}//namespace

#endif