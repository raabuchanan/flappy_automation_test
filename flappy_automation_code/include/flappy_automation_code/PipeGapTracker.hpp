#ifndef PIP_TRACKER_H_
#define PIP_TRACKER_H_

#include <iostream>

namespace flappy_navigation {


class PipeGapTracker {
  public:
	PipeGapTracker();
	~PipeGapTracker();

    void processUpdate(double x_displacement, double y_displacement);
    void measurementUpdate(double x_measurement, double y_measurement);
    double getXpos();
    double getYpos();

  private:

  double reInitializeKalmanFilter();

  double x_pos_;
  double y_pos_;

  double x_var_;
  double y_var_;

  bool uninitialized_;

  double process_noise_ = 0.01;
  double measurement_noise_ = 0.1; // should be related to resolution and distance

};

}//namespace

#endif