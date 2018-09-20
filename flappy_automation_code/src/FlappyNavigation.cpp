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

	node_handle_.param("p_gain", p_gain_, 1.0);
	node_handle_.param("d_gain", d_gain_, 15.0);
	node_handle_.param("i_gain", i_gain_, 0.001);
}


void FlappyNavigation::velCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
    if(last_vel_time_ == 0.0){
        last_vel_time_ = ros::Time::now().toSec();
        return;
    }

    double dt = 1 / 30.0;// ros::Time::now().toSec() - last_vel_time_;
    last_vel_time_ = ros::Time::now().toSec();

    //positionUpdate(0.0, msg->y * dt);
    tracker_.processUpdate(msg->x * dt, msg->y * dt);

    // std::cout << "dt " << dt << " " << std::endl;

    double displacement = y_goal_;

    integral_error_ += displacement;

    double y_cmd =  p_gain_*displacement + d_gain_*(displacement - prev_error_ ) + i_gain_*integral_error_;

    // std::cout << "P error " << displacement << " D error " << displacement - prev_error_ << " I error " << integral_error_ << std::endl;

    prev_error_ = displacement;

    geometry_msgs::Vector3 acc_cmd;
    acc_cmd.x = 0.0;
    acc_cmd.y = y_cmd;
    pub_acel_cmd_.publish(acc_cmd);




}

void FlappyNavigation::positionUpdate(double x_displacement, double y_displacement){

    y_pos_ += y_displacement;
    std::cout << "Bird position updated " << x_pos_ << " " << y_pos_ << std::endl;
}

void FlappyNavigation::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  int num_scans = (int)(abs(msg->angle_max - msg->angle_min)/msg->angle_increment) + 1;
//   std::cout << "Received " << num_scans << " laser scans" << std::endl;

  std::vector<std::vector<double>> depth_line;
  for(int i = 0; i < num_scans; i++){

    //if(msg->intensities[i] == 0.0) continue;

    std::vector<double> depth_measurement; //Laser or Robot centric?
    double x_pos = msg->ranges[i] * cos(msg->angle_min + i*msg->angle_increment);
    double y_pos = msg->ranges[i] * sin(msg->angle_min + i*msg->angle_increment);
    depth_measurement.push_back(x_pos);
    depth_measurement.push_back(y_pos);
    depth_measurement.push_back(msg->intensities[i]);

   // std::cout << "Point measurement " << x_pos << " " << y_pos << " " << msg->intensities[i] << std::endl;

    depth_line.push_back(depth_measurement);
  }


    map_mtx_.lock(); // necessary?
    depth_map_.clear();
    depth_map_ = depth_line;
    map_mtx_.unlock();

}


void FlappyNavigation::flappyControlCallback(const ros::TimerEvent&){

    double x_best = 0.0;
    std::vector<std::vector<double>> depth_line = depth_map_; //copy

        //RANSAC 1D line fitting
        int num_points = depth_line.size();
        double num_inliers = (int)0.7*num_points;
        int num_iterations = 5;
        double inlier_thresh = 0.25;//m
        int best_inliers = 0;

    if(depth_line.size() > 0){
        for (int k=0; k < num_iterations; k++){
            int rand_idx = rand() % num_points; // resampling
            double x_query = depth_line[k][0];
            int inliers = 0;
            for (int i=0; i < num_points; i++){
                if (abs(depth_line[i][0] - x_query) < inlier_thresh) inliers++;
            }

            if (inliers > best_inliers){
                x_best = x_query;
                best_inliers = inliers;
            }

        }
    }



    if(x_best == 0.0){
        std::cout << "Could not fit line" << std::endl;
        return;
    } else {
        std::cout << "Line fit to " << x_best << std::endl;
    }


    std::vector<double> y_gaps;
    double x_mean = 0.0;
    double x_variance = 0.0;
    for(const auto& point : depth_line){

        if(x_best - point[0] > inlier_thresh) continue; //floor or ceiling

        if(point[0] - x_best > inlier_thresh){ // gap
            std::cout << "Gap detected at " << point[0] << " " << point[1] << std::endl;
            y_gaps.push_back(point[1]);
        }
    }

    
    for(int j = 0; j < y_gaps.size(); j++){
        tracker_.measurementUpdate(x_best, y_gaps[j]);
    }


    
    y_goal_ = tracker_.getYpos(); // keep centered

    std::cout << "y_goal_ " << y_goal_ << std::endl;

}


}// namespace