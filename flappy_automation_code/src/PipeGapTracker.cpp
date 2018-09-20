#include "flappy_automation_code/PipeGapTracker.hpp"


namespace flappy_navigation {

PipeGapTracker::PipeGapTracker(){
    reInitializeKalmanFilter();
}

PipeGapTracker::~PipeGapTracker() {
}

double PipeGapTracker::reInitializeKalmanFilter(){
    x_pos_ = 3.0;
    y_pos_ = 0.0;
    x_var_ = 0.01;
    y_var_ = 0.01;
    uninitialized_ = true;
}

double PipeGapTracker::getXpos(){
    return x_pos_;
}

double PipeGapTracker::getYpos(){
    return y_pos_;
}

void PipeGapTracker::processUpdate(double x_displacement, double y_displacement){
    x_pos_ = x_pos_ + x_displacement;
    x_var_ = x_var_ + process_noise_;

    y_pos_ = y_pos_ + y_displacement;
    y_var_ = y_var_ + process_noise_;

}


void PipeGapTracker::measurementUpdate(double x_measurement, double y_measurement){

    // if(!uninitialized_){

        
    // }

    double x_innovation = x_measurement - x_pos_;
    double x_innovation_var = measurement_noise_ + x_var_;
    double x_kalman_gain = x_var_ / x_innovation_var;

    double y_innovation = y_measurement - y_pos_;
    double y_innovation_var = measurement_noise_ + y_var_;
    double y_kalman_gain = y_var_ / y_innovation_var;


    x_pos_ = x_pos_ + x_kalman_gain*x_innovation;
    y_pos_ = y_pos_ + y_kalman_gain*y_innovation;

    x_var_ = (1 - x_kalman_gain)*(1 - x_kalman_gain)*x_var_ + x_kalman_gain*x_kalman_gain*measurement_noise_;
    y_var_ = (1 - y_kalman_gain)*(1 - y_kalman_gain)*y_var_ + y_kalman_gain*y_kalman_gain*measurement_noise_;

    std::cout << "Gap position: " << x_pos_ << " " << y_pos_ << " variance: " << x_var_ << " " << y_var_ << std::endl;

    if(x_pos_ < 0.2) {
        std::cout << "[REINITIALIZING] " << std::endl;
        reInitializeKalmanFilter();

    } 
}


}// namespace