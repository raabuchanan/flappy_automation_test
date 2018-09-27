#include "flappy_automation_code/FlappyNavigation.hpp"

namespace flappy_navigation {

FlappyNavigation::FlappyNavigation(ros::NodeHandle& node_handle):
	node_handle_(node_handle), last_vel_time_(0.0), x_pos_(0.0), y_pos_(0.0), y_goal_(0.0), integral_error_(0.0), prev_error_(0.0){

        readParameters();

  //Init publishers and subscribers
  pub_acel_cmd_ = node_handle_.advertise<geometry_msgs::Vector3>("/flappy_acc",1);
  sub_vel_ = node_handle_.subscribe<geometry_msgs::Vector3>("/flappy_vel", 1, &FlappyNavigation::velCallback, this);
  sub_laser_scan_ = node_handle_.subscribe<sensor_msgs::LaserScan>("/flappy_laser_scan", 1, &FlappyNavigation::laserScanCallback, this);
  
  flappy_navigation_duration_.fromSec(0.05);//20hz
  flappy_navigation_timer_ = node_handle_.createTimer(flappy_navigation_duration_, &FlappyNavigation::flappyControlCallback, this);
}

FlappyNavigation::~FlappyNavigation() {
}


void FlappyNavigation::readParameters(){

    // Load parameters from ros server
	node_handle_.param("/flappy_automation_code/p_gain", p_gain_, 1.0);
	node_handle_.param("/flappy_automation_code/d_gain", d_gain_, 15.0);
	node_handle_.param("/flappy_automation_code/i_gain", i_gain_, 0.001);
    node_handle_.param("/flappy_automation_code/x_vel", x_vel_, 0.5);
}


void FlappyNavigation::velCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
    if(last_vel_time_ == 0.0){
        last_vel_time_ = ros::Time::now().toSec();
        return;
    }

    //TODO
    double dt = ros::Time::now().toSec() - last_vel_time_;
    last_vel_time_ = ros::Time::now().toSec();

    first_gap_.processUpdate(-msg->x * dt, -msg->y * dt);
    second_gap_.processUpdate(-msg->x * dt, -msg->y * dt);

    if(first_gap_.getXpos() <= second_gap_.getXpos()){
        y_goal_ = first_gap_.getYpos();
        if(first_gap_.getXpos() < 0.25 || first_gap_.getXpos() > 1.5) y_goal_ = 0.0;
        ROS_INFO_STREAM("Tracking First Gap " << first_gap_.getXpos() << " " << first_gap_.getYpos());
    } else {
        y_goal_ = second_gap_.getYpos();
        if(second_gap_.getXpos() < 0.25 || second_gap_.getXpos() > 1.5) y_goal_ = 0.0;
        ROS_INFO_STREAM("Tracking Second Gap " << second_gap_.getXpos() << " " << second_gap_.getYpos());
    }

    double proportional_error_ = y_goal_;//Birdy-centric
    double y_cmd =  p_gain_*proportional_error_ + d_gain_*(proportional_error_ - prev_error_ ) + i_gain_*integral_error_;
    integral_error_ += proportional_error_;
    prev_error_ = proportional_error_;

    if (y_goal_ == 0.0) y_cmd = (0.0 - msg->y) / dt; // zero y velocity

    double desired_vel = x_vel_;
    if (proportional_error_ <= 0.01){
        desired_vel = x_vel_ / abs(proportional_error_); // move slower if more than 1m away
    }
    if(desired_vel > x_vel_) desired_vel = x_vel_;

    double x_cmd = (desired_vel - msg->x) / dt;

    geometry_msgs::Vector3 acc_cmd;
    acc_cmd.x = x_cmd;
    acc_cmd.y = y_cmd;
    pub_acel_cmd_.publish(acc_cmd);
}

void FlappyNavigation::positionUpdate(double x_displacement, double y_displacement){
    y_pos_ += y_displacement;
}

void FlappyNavigation::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  int num_scans = (int)(fabs(msg->angle_max - msg->angle_min)/msg->angle_increment);
  std::vector<std::vector<double>> depth_line;
  for(int i = 0; i < num_scans; i++){

    std::vector<double> depth_measurement; //Birdy-centric
    double x_pos = msg->ranges[i] * cos(msg->angle_min + i*msg->angle_increment);
    double y_pos = msg->ranges[i] * sin(msg->angle_min + i*msg->angle_increment);
    depth_measurement.push_back(x_pos);
    depth_measurement.push_back(y_pos);
    depth_measurement.push_back(msg->intensities[i]);

    depth_line.push_back(depth_measurement);
  }
    angle_resolution_ = msg->angle_increment;

    map_mtx_.lock(); // necessary?
    depth_map_.clear();
    depth_map_ = depth_line;
    map_mtx_.unlock();

}


void FlappyNavigation::flappyControlCallback(const ros::TimerEvent&){

    double x_best = 0.0;
    std::vector<std::vector<double>> depth_line = depth_map_; //copy

    //RANSAC 1D line fitting
    // TODO: Parametrize
    int num_points = depth_line.size();
    double num_inliers = (int)0.7*num_points;
    int num_iterations = 10;
    double inlier_thresh = 0.25;//m
    int best_inliers = 0;

    if(depth_line.size() > 0){
        for (int k=0; k < num_iterations; k++){
            int rand_idx = rand() % num_points; // sample with replacement
            double x_query = depth_line[rand_idx][0];
            int inliers = 0;
            for (int i=0; i < num_points; i++){
                if (fabs(depth_line[i][0] - x_query) < inlier_thresh) inliers++;
            }

            if (inliers > best_inliers){
                x_best = x_query;
                best_inliers = inliers;
            }
        }
    }

    if(x_best == 0.0){
        ROS_WARN("Could not fit line");
        return;
    }

    std::vector<double> y_gaps;
    double x_mean = 0.0;
    double x_variance = 0.0;
    for(const auto& point : depth_line){

        if(x_best - point[0] > inlier_thresh) continue; //floor or ceiling

        if(point[0] - x_best > inlier_thresh){ // gap
            // ROS_INFO_STREAM("Gap detected at " << point[0] << " " << point[1]);
            y_gaps.push_back(point[1]);
        }
    }

    if(!first_gap_.getInitialization()){
        first_gap_.measurementUpdate(x_best, y_gaps, angle_resolution_);
    } else if (fabs(x_best - first_gap_.getXpos()) > fabs(x_best - second_gap_.getXpos()) ) {
        second_gap_.measurementUpdate(x_best, y_gaps, angle_resolution_);
    } else {
        first_gap_.measurementUpdate(x_best, y_gaps, angle_resolution_);
    }

}


}// namespace