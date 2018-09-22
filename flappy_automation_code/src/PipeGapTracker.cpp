#include "flappy_automation_code/PipeGapTracker.hpp"


namespace flappy_navigation {

PipeGapTracker::PipeGapTracker(){
    reInitializeKalmanFilter();
}

PipeGapTracker::~PipeGapTracker() {
}

double PipeGapTracker::reInitializeKalmanFilter(){
    x_pos_ = 3.5;
    y_pos_ = 0.0;
    x_var_ = 0.01;
    y_var_ = 0.01;
    initialized_ = false;
}

double PipeGapTracker::getXpos(){
    return x_pos_;
}

double PipeGapTracker::getYpos(){
    return y_pos_;
}

void PipeGapTracker::processUpdate(double x_displacement, double y_displacement){
    if(!initialized_){
        return;        
    }

    x_pos_ = x_pos_ + x_displacement;
    x_var_ = x_var_ + process_noise_;

    y_pos_ = y_pos_ + y_displacement;
    y_var_ = y_var_ + process_noise_;

    if(x_pos_ < -0.3) {
        ROS_WARN("Reinitializing Kalman Filter");
        reInitializeKalmanFilter();
    }

}


void PipeGapTracker::measurementUpdate(double x_measurement, std::vector<double> y_measurements, double resolution){

    if(y_measurements.size() == 0) return;

    // Variance based on resolution
    // Actual laser measuremnet noise is proportional to depth^2
    double measurement_noise = x_measurement * sin(resolution) * x_measurement * sin(resolution) / 4.0;

    //Don't track the next gap too soon
    if( x_measurement >= 1.5){
        measurement_noise *=y_measurements.size()*y_measurements.size();
    }

    for (const auto& y_measurement : y_measurements){
        double x_innovation = x_measurement - x_pos_;
        double x_innovation_var = measurement_noise + x_var_;
        double x_kalman_gain = x_var_ / x_innovation_var;

        double y_innovation = y_measurement - y_pos_;
        double y_innovation_var = measurement_noise + y_var_;
        double y_kalman_gain = y_var_ / y_innovation_var;


        x_pos_ = x_pos_ + x_kalman_gain*x_innovation;
        y_pos_ = y_pos_ + y_kalman_gain*y_innovation;

        x_var_ = (1 - x_kalman_gain)*(1 - x_kalman_gain)*x_var_ + x_kalman_gain*x_kalman_gain*measurement_noise;
        y_var_ = (1 - y_kalman_gain)*(1 - y_kalman_gain)*y_var_ + y_kalman_gain*y_kalman_gain*measurement_noise;
    }
    initialized_ = true;
}

bool PipeGapTracker::getInitialization(){
    return initialized_;
}

}// namespace