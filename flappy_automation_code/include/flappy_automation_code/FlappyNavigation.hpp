#ifndef FLAPPY_NAVIGATION_H_
#define FLAPPY_NAVIGATION_H_


#include "ros/ros.h"
#include "flappy_automation_code/PipeGapTracker.hpp"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Vector3.h"
#include <iostream>

#include <mutex>
namespace flappy_navigation {


class FlappyNavigation {
  public:
	FlappyNavigation(ros::NodeHandle& node_handle);
	~FlappyNavigation();
  private:

    //Ros nodehandle
    ros::NodeHandle node_handle_;
    //Publisher for acceleration command
    ros::Publisher pub_acel_cmd_;
    //Subscriber for velocity
    ros::Subscriber sub_vel_;
    //Subscriber for laser scan
    ros::Subscriber sub_laser_scan_;

    PipeGapTracker tracker_;

    ros::Timer flappy_navigation_timer_;
    ros::Duration flappy_navigation_duration_;

    double last_vel_time_;
    std::mutex map_mtx_;

    double y_goal_;
    double integral_error_;
    double p_gain_;
    double i_gain_;
    double d_gain_;
    double prev_error_;

    std::vector<std::vector<double>> depth_map_;

    void velCallback(const geometry_msgs::Vector3::ConstPtr& msg);
    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void positionUpdate(double x_displacement, double y_displacement);
    void flappyControlCallback(const ros::TimerEvent&);
    void readParameters();

    double x_pos_;
    double y_pos_;

};

}//namespace

#endif